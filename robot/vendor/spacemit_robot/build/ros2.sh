#!/usr/bin/env bash
#
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#

# ROS2 build logic (colcon).
#
# This file is meant to be SOURCED by build scripts (e.g. build/build.sh).
#
# Responsibilities:
# - ROS2 env loading
# - colcon build for middleware + application workspaces
# - rootfs deploy

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[build] ERROR: build/ros2.sh must be sourced, not executed." >&2
  exit 1
fi

# Ensure dependencies are loaded when sourced directly.
if ! command -v map_config_path_to_ros2_package >/dev/null 2>&1; then
  if [[ -z "${REPO_ROOT:-}" ]]; then
    REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  fi
  source "${REPO_ROOT}/build/common.sh"
fi
if ! command -v build_nonros2_enabled_packages >/dev/null 2>&1; then
  source "${REPO_ROOT}/build/nonros2.sh"
fi

load_ros2_env() {
  if [[ ! -f "${ROS_SETUP}" ]]; then
    if [[ "${SROBOTIS_CROSS_BUILD:-0}" == "1" ]]; then
      echo "[build] ROS setup script not found on host, continuing in cross-build mode: ${ROS_SETUP}"
      return 0
    fi
    echo "[build] ERROR: ROS setup script not found: ${ROS_SETUP}" >&2
    exit 1
  fi
  set +u
  # shellcheck source=/dev/null
  source "${ROS_SETUP}"
  set -u
}

build_ros2_workspace() {
  local ws_dir="$1"
  local build_base="$2"
  local log_base="$3"
  local packages=("${@:4}")
  local package_mode="${SROBOTIS_ROS2_PACKAGE_DEPS_MODE:-select}"

  if [[ ! -d "${ws_dir}" ]]; then
    echo "[build] ROS2 workspace not found, skipping: ${ws_dir}"
    return 0
  fi

  # When building the full workspace (m -R / m), ensure non-ROS2 packages are built first.
  # When building a single package (mm), skip this so only that ROS2 package is built.
  if [[ ${#packages[@]} -eq 0 && -n "${BUILD_CONFIG_FILE:-}" && -f "${BUILD_CONFIG_FILE:-/dev/null}" ]]; then
    build_nonros2_enabled_packages
  fi

  load_ros2_env

  mkdir -p "${build_base}" "${log_base}"
  cd "${ws_dir}" || return 1

  local colcon_args=(
    --merge-install
    --install-base "${PREFIX}"
    --build-base "${build_base}"
    --parallel-workers "${PARALLEL_JOBS}"
    --cmake-args -DCMAKE_PREFIX_PATH="${PREFIX}"
  )

  if [[ -n "${SROBOTIS_ROS2_CMAKE_EXTRA_ARGS:-}" ]]; then
    while IFS= read -r line; do
      [[ -n "${line}" ]] && colcon_args+=("${line}")
    done <<< "${SROBOTIS_ROS2_CMAKE_EXTRA_ARGS}"
  fi

  if [[ -n "${SROBOTIS_CMAKE_EXTRA_ARGS:-}" ]]; then
    while IFS= read -r line; do
      [[ -n "${line}" ]] && colcon_args+=("${line}")
    done <<< "${SROBOTIS_CMAKE_EXTRA_ARGS}"
  fi

  if [[ ${#packages[@]} -gt 0 ]]; then
    case "${package_mode}" in
      with)
        colcon_args+=(--packages-up-to "${packages[@]}")
        ;;
      only)
        local dep_packages=()
        local up_to_packages=()
        local up_to_output
        if ! up_to_output="$(colcon list --packages-up-to "${packages[@]}" --names-only)"; then
          echo "[build] ERROR: Failed to resolve ROS2 dependencies for package: ${packages[*]}" >&2
          return 1
        fi
        mapfile -t up_to_packages <<< "${up_to_output}"
        local p selected
        for p in "${up_to_packages[@]}"; do
          [[ -n "${p}" ]] || continue
          selected=0
          local root_pkg
          for root_pkg in "${packages[@]}"; do
            if [[ "${p}" == "${root_pkg}" ]]; then
              selected=1
              break
            fi
          done
          [[ "${selected}" == "1" ]] && continue
          dep_packages+=("${p}")
        done
        if [[ ${#dep_packages[@]} -eq 0 ]]; then
          echo "[build] No ROS2 dependencies to build for package: ${packages[*]}"
          return 0
        fi
        packages=("${dep_packages[@]}")
        colcon_args+=(--packages-select "${packages[@]}")
        ;;
      select|"")
        colcon_args+=(--packages-select "${packages[@]}")
        ;;
      *)
        echo "[build] ERROR: Unknown ROS2 package dependency mode: ${package_mode}" >&2
        return 1
        ;;
    esac
  else
    :
  fi

  if [[ "${LOG_SHOW_ENTER}" == "1" ]]; then
    local label="ROS2"
    if [[ "${ws_dir}" == */middleware/ros2 ]]; then
      label="ROS2 middleware"
    elif [[ "${ws_dir}" == */application/ros2 ]]; then
      label="ROS2 applications"
    fi

    if [[ ${#packages[@]} -eq 0 ]]; then
      echo "[build] Start: ${label} (all)"
    else
      local p
      for p in "${packages[@]}"; do
        [[ -n "${p}" ]] && echo "[build] Start: ${label} pkg ${p}"
      done
    fi
  fi

  mkdir -p "${log_base}"
  local console_log="${log_base}/colcon-console.log"
  local parallel_jobs="${PARALLEL_JOBS:-$(nproc)}"
  if [[ ! "${parallel_jobs}" =~ ^[1-9][0-9]*$ ]]; then
    echo "[build] ERROR: invalid PARALLEL_JOBS for ROS2 build: ${parallel_jobs}" >&2
    return 1
  fi

  local makeflags="${MAKEFLAGS:-}"
  if [[ ! "${makeflags}" =~ (^|[[:space:]])(-?j[0-9]*|--?jobs([=[:space:]]|$)|-?l[0-9]*|--?load-average([=[:space:]]|$))([[:space:]]|$) ]]; then
    makeflags="-j${parallel_jobs} -l${parallel_jobs}${makeflags:+ ${makeflags}}"
  fi
  local colcon_env=(
    "COLCON_LOG_PATH=${log_base}"
    "CMAKE_BUILD_PARALLEL_LEVEL=${parallel_jobs}"
    "MAKEFLAGS=${makeflags}"
  )

  if [[ "${LOG_LEVEL}" == "verbose" ]]; then
    if [[ "${LOG_TEE}" == "1" ]]; then
      env "${colcon_env[@]}" colcon build "${colcon_args[@]}" 2>&1 | tee "${console_log}"
      return "${PIPESTATUS[0]}"
    fi
    env "${colcon_env[@]}" colcon build "${colcon_args[@]}"
    return $?
  fi

  # quiet/normal: keep console clean; write full output to a file.
  set +e
  env "${colcon_env[@]}" colcon build "${colcon_args[@]}" >"${console_log}" 2>&1
  local rc=$?
  set -e
  if [[ "${rc}" -ne 0 ]]; then
    echo "[build] ERROR: ROS2 build failed (rc=${rc}). See log: ${console_log}" >&2
    tail -n "$(_log_tail_lines)" "${console_log}" >&2 || true
    return "${rc}"
  fi
}

deploy_rootfs() {
  local src="${STAGING_PREFIX}"
  local dst="${ROOTFS_PREFIX}"

  if [[ ! -d "${src}" ]]; then
    echo "[build] ERROR: staging prefix does not exist: ${src}" >&2
    echo "[build]        Build first (e.g. ./build/build.sh cmake / ros2 / all)." >&2
    return 1
  fi

  mkdir -p "${dst}"

  echo "[build] Deploy rootfs: ${src} -> ${dst}"
  if command -v rsync >/dev/null 2>&1; then
    rsync -a --delete "${src}/" "${dst}/"
  else
    rm -rf "${dst:?}/"* 2>/dev/null || true
    cp -a "${src}/." "${dst}/"
  fi

  rm -rf "${dst}/include" "${dst}/lib/cmake" "${dst}/lib/pkgconfig" 2>/dev/null || true

  if [[ -d "${dst}/share" ]]; then
    find "${dst}/share" -maxdepth 2 -type d -name cmake -prune -exec rm -rf {} + 2>/dev/null || true
  fi

  find "${dst}" -type f \( -name '*.a' -o -name '*.la' \) -delete 2>/dev/null || true

  echo "[build] Rootfs ready at: ${dst}"
}

build_ros2_middleware() {
  local packages=("$@")
  local config_file="${BUILD_CONFIG_FILE:-}"

  if [[ ! -d "${REPO_ROOT}/middleware/ros2" ]]; then
    [[ ${#packages[@]} -eq 0 ]] && echo "[build] No middleware/ros2 workspace; skipping ROS2 middleware."
    return 0
  fi

  if [[ ${#packages[@]} -eq 0 && -n "${config_file}" && -f "${config_file}" ]]; then
    local enabled_packages=()
    local all_enabled=()
    mapfile -t all_enabled < <(resolve_enabled_with_metadata)
    for p in "${all_enabled[@]}"; do
      [[ "${p}" == middleware/* ]] && enabled_packages+=("${p#middleware/}")
    done

    local ros2_packages=()
    for pkg_path in "${enabled_packages[@]}"; do
      if [[ "${pkg_path}" == ros2/* ]]; then
        local rel_path="${pkg_path#ros2/}"
        local mapped_packages
        mapfile -t mapped_packages < <(map_config_path_to_ros2_package "middleware/ros2" "${rel_path}" 2>/dev/null)
        for actual_pkg_name in "${mapped_packages[@]}"; do
          [[ -n "${actual_pkg_name}" ]] && ros2_packages+=("${actual_pkg_name}")
        done
      fi
    done

    packages=("${ros2_packages[@]}")
    if [[ ${#packages[@]} -eq 0 ]]; then
      echo "[build] No enabled ROS2 middleware packages in configuration"
      return 0
    fi
  fi

  build_ros2_workspace \
    "${REPO_ROOT}/middleware/ros2" \
    "${ROS2_BUILD_ROOT}/middleware" \
    "${ROS2_LOG_ROOT}/middleware" \
    "${packages[@]}"
  local rc=$?
  if [[ "${rc}" -eq 0 && "${LOG_LEVEL:-quiet}" != "verbose" && "${LOG_SHOW_DONE:-1}" == "1" ]]; then
    if [[ ${#packages[@]} -eq 0 ]]; then
      echo "[build] End: ROS2 middleware (all)"
    else
      local p
      for p in "${packages[@]}"; do
        [[ -n "${p}" ]] && echo "[build] End: ROS2 middleware pkg ${p}"
      done
    fi
  fi
  return "${rc}"
}

build_ros2_applications() {
  local packages=("$@")
  local config_file="${BUILD_CONFIG_FILE:-}"

  if [[ ! -d "${REPO_ROOT}/application/ros2" ]]; then
    [[ ${#packages[@]} -eq 0 ]] && echo "[build] No application/ros2 workspace; skipping ROS2 applications."
    return 0
  fi

  if [[ ${#packages[@]} -eq 0 && -n "${config_file}" && -f "${config_file}" ]]; then
    local enabled_packages=()
    local all_enabled=()
    mapfile -t all_enabled < <(resolve_enabled_with_metadata)
    for p in "${all_enabled[@]}"; do
      [[ "${p}" == application/* ]] && enabled_packages+=("${p#application/}")
    done

    local ros2_packages=()
    for pkg_path in "${enabled_packages[@]}"; do
      if [[ "${pkg_path}" == ros2/* ]]; then
        local rel_path="${pkg_path#ros2/}"
        local mapped_packages
        mapfile -t mapped_packages < <(map_config_path_to_ros2_package "application/ros2" "${rel_path}" 2>/dev/null)
        for actual_pkg_name in "${mapped_packages[@]}"; do
          [[ -n "${actual_pkg_name}" ]] && ros2_packages+=("${actual_pkg_name}")
        done
      fi
    done

    packages=("${ros2_packages[@]}")
    if [[ ${#packages[@]} -eq 0 ]]; then
      echo "[build] No enabled ROS2 application packages in configuration"
      return 0
    fi
  fi

  build_ros2_workspace \
    "${REPO_ROOT}/application/ros2" \
    "${ROS2_BUILD_ROOT}/application" \
    "${ROS2_LOG_ROOT}/application" \
    "${packages[@]}"
  local rc=$?
  if [[ "${rc}" -eq 0 && "${LOG_LEVEL:-quiet}" != "verbose" && "${LOG_SHOW_DONE:-1}" == "1" ]]; then
    if [[ ${#packages[@]} -eq 0 ]]; then
      echo "[build] End: ROS2 applications (all)"
    else
      local p
      for p in "${packages[@]}"; do
        [[ -n "${p}" ]] && echo "[build] End: ROS2 applications pkg ${p}"
      done
    fi
  fi
  return "${rc}"
}
