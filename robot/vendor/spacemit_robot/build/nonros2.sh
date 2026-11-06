#!/usr/bin/env bash
#
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#

# Non-ROS2 (per-package) build logic.
#
# This file is meant to be SOURCED by build scripts (e.g. build/build.sh).
#
# Responsibilities:
# - per-package build dir selection
# - thirdparty overlay source selection
# - deps-aware scheduler (parallel build + serialized install)
# - generate ComponentsConfig.cmake for ROS2 consumers

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[build] ERROR: build/nonros2.sh must be sourced, not executed." >&2
  exit 1
fi

# Ensure common helpers are loaded when sourced directly.
if ! command -v read_package_deps >/dev/null 2>&1; then
  if [[ -z "${REPO_ROOT:-}" ]]; then
    REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  fi
  source "${REPO_ROOT}/build/common.sh"
fi

# Python wheel helper (optional PEP 517 wheels under output/wheels/)
# shellcheck source=build/python_wheels.sh
source "${REPO_ROOT}/build/python_wheels.sh"

pkg_key_to_dir() {
  local pkg_key="$1"
  echo "${REPO_ROOT}/${pkg_key}"
}

pkg_overlay_src_dir() {
  local pkg_key="$1"
  local pkg_dir
  pkg_dir="$(pkg_key_to_dir "${pkg_key}")"

  # Overlay is only supported for components/thirdparty/*.
  # Use directory convention: if platform overlay dir has CMakeLists.txt or build.sh, use it.
  if [[ "${pkg_key}" == components/thirdparty/* ]]; then
    local name="${pkg_key#components/thirdparty/}"
    local overlay_dir="${REPO_ROOT}/platform/generic/components/thirdparty/${name}"
    if [[ -f "${overlay_dir}/CMakeLists.txt" || -f "${overlay_dir}/build.sh" ]]; then
      echo "${overlay_dir}"
      return 0
    fi
  fi

  echo "${pkg_dir}"
}

# Classify by path only. Components may have package.xml for dependency metadata but are built as non-ROS2.
pkg_key_is_ros2() {
  local pkg_key="$1"
  [[ "${pkg_key}" == middleware/ros2/* || "${pkg_key}" == application/ros2/* ]]
}

pkg_build_dir() {
  local pkg_key="$1"
  local safe
  safe="$(echo "${pkg_key}" | tr '/ ' '__')"
  echo "${CM_BUILD_ROOT}/pkgs/${safe}"
}

pkg_cmake_extra_args() {
  local pkg_key="$1"

  # Peripherals driver selection (from target enabled_package_options).
  if [[ "${pkg_key}" =~ ^components/peripherals/([^/]+)$ ]]; then
    local mod="${BASH_REMATCH[1]}"
    local mod_upper
    mod_upper="$(echo "${mod}" | tr '[:lower:]-' '[:upper:]_')"
    local drivers=""
    drivers="$(get_target_option_list "${pkg_key}" '.enabled_drivers[]? // empty' | paste -sd ';' - || true)"
    local args="-DSROBOTIS_PERIPHERALS_${mod_upper}_ENABLED_DRIVERS=${drivers}"
    echo "${args}"
  fi
}

with_install_lock() {
  local lock_file="${OUTPUT_ROOT}/.install.lock"
  mkdir -p "${OUTPUT_ROOT}"

  if command -v flock >/dev/null 2>&1; then
    exec 9>"${lock_file}"
    flock 9
    "$@"
    local rc=$?
    flock -u 9 || true
    exec 9>&- || true
    return "${rc}"
  fi

  local lock_dir="${lock_file}.d"
  while ! mkdir "${lock_dir}" 2>/dev/null; do
    sleep 0.1
  done
  "$@"
  local rc=$?
  rmdir "${lock_dir}" 2>/dev/null || true
  return "${rc}"
}

build_one_nonros2_pkg() {
  local pkg_key="$1"
  local want_py_wheels="${2:-0}"
  local pkg_dir
  pkg_dir="$(pkg_key_to_dir "${pkg_key}")"

  local safe
  safe="$(echo "${pkg_key}" | tr '/ ' '__')"
  local log_file="${LOG_ROOT}/cmake/pkgs/${safe}.log"
  local in_sched="${SROBOTIS_IN_SCHED:-0}"

  # For "single package" builds (mm / build.sh package), print Start/End here.
  # For scheduler builds, Start/End are printed by the scheduler to avoid duplicates.
  if [[ "${in_sched}" != "1" && "${LOG_SHOW_ENTER}" == "1" ]]; then
    echo "[build] Start: ${pkg_key}"
  fi

  # Custom per-package build script (check overlay dir first, then real package dir).
  local build_sh_dir=""
  local overlay_dir
  overlay_dir="$(pkg_overlay_src_dir "${pkg_key}")"
  if [[ -f "${overlay_dir}/build.sh" && -x "${overlay_dir}/build.sh" ]]; then
    build_sh_dir="${overlay_dir}"
  elif [[ -d "${pkg_dir}" && -f "${pkg_dir}/build.sh" && -x "${pkg_dir}/build.sh" ]]; then
    build_sh_dir="${pkg_dir}"
  fi

  if [[ -n "${build_sh_dir}" ]]; then
    if [[ "${LOG_LEVEL}" == "verbose" ]]; then
      if [[ "${LOG_TEE}" == "1" ]]; then
        mkdir -p "$(dirname "${log_file}")"
        with_install_lock bash -lc "
          set -euo pipefail
          cd \"${build_sh_dir}\"
          export PREFIX=\"${PREFIX}\"
          export CMAKE_PREFIX_PATH=\"${PREFIX}\"
          export PARALLEL_JOBS=\"${PARALLEL_JOBS}\"
          ./build.sh
        " 2>&1 | tee "${log_file}"
        local rc="${PIPESTATUS[0]}"
        if [[ "${rc}" -eq 0 ]]; then
          srobotis_maybe_build_python_wheel "${pkg_key}" "${pkg_dir}" "${want_py_wheels}" || return $?
        fi
        if [[ "${rc}" -eq 0 && "${in_sched}" != "1" && "${LOG_SHOW_DONE}" == "1" ]]; then
          echo "[build] End: ${pkg_key}"
        fi
        return "${rc}"
      fi
      with_install_lock bash -lc "
        set -euo pipefail
        cd \"${build_sh_dir}\"
        export PREFIX=\"${PREFIX}\"
        export CMAKE_PREFIX_PATH=\"${PREFIX}\"
        export PARALLEL_JOBS=\"${PARALLEL_JOBS}\"
        ./build.sh
      "
      local rc=$?
      if [[ "${rc}" -eq 0 ]]; then
        srobotis_maybe_build_python_wheel "${pkg_key}" "${pkg_dir}" "${want_py_wheels}" || return $?
      fi
      if [[ "${rc}" -eq 0 && "${in_sched}" != "1" && "${LOG_SHOW_DONE}" == "1" ]]; then
        echo "[build] End: ${pkg_key}"
      fi
      return "${rc}"
    fi

    mkdir -p "$(dirname "${log_file}")"
    with_install_lock bash -lc "
      set -euo pipefail
      cd \"${build_sh_dir}\"
      export PREFIX=\"${PREFIX}\"
      export CMAKE_PREFIX_PATH=\"${PREFIX}\"
      export PARALLEL_JOBS=\"${PARALLEL_JOBS}\"
      ./build.sh
    " >"${log_file}" 2>&1 || {
      local rc=$?
      echo "[build] ERROR: Package build failed: ${pkg_key}. See log: ${log_file}" >&2
      tail -n "$(_log_tail_lines)" "${log_file}" >&2 || true
      return "${rc}"
    }
    srobotis_maybe_build_python_wheel "${pkg_key}" "${pkg_dir}" "${want_py_wheels}" || return $?
    if [[ "${in_sched}" != "1" && "${LOG_SHOW_DONE}" == "1" ]]; then
      echo "[build] End: ${pkg_key}"
    fi
    return 0
  fi

  # CMake package (may be an overlay wrapper under platform/generic).
  local src_dir
  src_dir="$(pkg_overlay_src_dir "${pkg_key}")"
  if [[ -f "${src_dir}/CMakeLists.txt" ]]; then
    local bdir
    bdir="$(pkg_build_dir "${pkg_key}")"
    mkdir -p "${bdir}"

    local extra
    extra="$(pkg_cmake_extra_args "${pkg_key}")"
    # Extra CMake -D options from mm (e.g. mm -DBUILD_STREAM_DEMO=ON)
    local cmake_user_args=()
    if [[ -n "${SROBOTIS_CMAKE_EXTRA_ARGS:-}" ]]; then
      while IFS= read -r line; do
        [[ -n "${line}" ]] && cmake_user_args+=("${line}")
      done <<< "${SROBOTIS_CMAKE_EXTRA_ARGS}"
    fi

    if [[ -n "${extra}" ]]; then
      # shellcheck disable=SC2086
      run_logged_overwrite "${log_file}" cmake -S "${src_dir}" -B "${bdir}" \
        -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
        -DCMAKE_PREFIX_PATH="${PREFIX}" \
        ${extra} "${cmake_user_args[@]}"
    else
      run_logged_overwrite "${log_file}" cmake -S "${src_dir}" -B "${bdir}" \
        -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
        -DCMAKE_PREFIX_PATH="${PREFIX}" \
        "${cmake_user_args[@]}"
    fi

    run_logged_append "${log_file}" cmake --build "${bdir}" -j"${PARALLEL_JOBS}"
    # Serialize installs to avoid concurrent writes under PREFIX.
    if [[ "${LOG_LEVEL}" == "verbose" && "${LOG_TEE}" == "1" ]]; then
      with_install_lock cmake --install "${bdir}" 2>&1 | tee -a "${log_file}"
      local rc="${PIPESTATUS[0]}"
      if [[ "${rc}" -eq 0 ]]; then
        srobotis_maybe_build_python_wheel "${pkg_key}" "${pkg_dir}" "${want_py_wheels}" || return $?
      fi
      if [[ "${rc}" -eq 0 && "${in_sched}" != "1" && "${LOG_SHOW_DONE}" == "1" ]]; then
        echo "[build] End: ${pkg_key}"
      fi
      return "${rc}"
    fi
    if [[ "${LOG_LEVEL}" == "verbose" ]]; then
      with_install_lock cmake --install "${bdir}"
      local rc=$?
      if [[ "${rc}" -eq 0 ]]; then
        srobotis_maybe_build_python_wheel "${pkg_key}" "${pkg_dir}" "${want_py_wheels}" || return $?
      fi
      if [[ "${rc}" -eq 0 && "${in_sched}" != "1" && "${LOG_SHOW_DONE}" == "1" ]]; then
        echo "[build] End: ${pkg_key}"
      fi
      return "${rc}"
    fi
    with_install_lock cmake --install "${bdir}" >>"${log_file}" 2>&1 || {
      local rc=$?
      echo "[build] ERROR: Package install failed: ${pkg_key}. See log: ${log_file}" >&2
      tail -n "$(_log_tail_lines)" "${log_file}" >&2 || true
      return "${rc}"
    }
    srobotis_maybe_build_python_wheel "${pkg_key}" "${pkg_dir}" "${want_py_wheels}" || return $?
    if [[ "${in_sched}" != "1" && "${LOG_SHOW_DONE}" == "1" ]]; then
      echo "[build] End: ${pkg_key}"
    fi
    return 0
  fi

  if [[ ! -d "${pkg_dir}" ]]; then
    echo "[build] ERROR: Package dir not found: ${pkg_dir} (pkg_key=${pkg_key})" >&2
    return 1
  fi

  echo "[build] ERROR: Unsupported non-ROS2 package (need CMakeLists.txt or build.sh): ${pkg_dir}" >&2
  return 1
}

build_nonros2_enabled_packages() {
  # If we already built non-ROS2 packages in this *same* build.sh invocation,
  # skip re-running to avoid duplicated work/log spam (ROS2 build may call us).
  if [[ "${SROBOTIS_NONROS2_BUILT:-0}" == "1" ]] && \
     [[ "${SROBOTIS_NONROS2_BUILT_CONFIG:-}" == "${BUILD_CONFIG_FILE:-}" ]] && \
     [[ "${SROBOTIS_NONROS2_BUILT_PREFIX:-}" == "${PREFIX}" ]]; then
    if [[ "${LOG_LEVEL:-quiet}" != "quiet" ]]; then
      echo "[build] Non-ROS2 packages already built for this invocation; skipping."
    fi
    return 0
  fi

  local want_py_wheels="${1:-0}"

  local config_file="${BUILD_CONFIG_FILE:-}"
  local no_config=0

  # If no target config is selected, fall back to building all buildable non-ROS2 packages in the repo.
  # This keeps ./build/build.sh cmake usable without requiring lunch/BUILD_TARGET.
  local enabled_all=()
  if [[ -z "${config_file}" || ! -f "${config_file}" ]]; then
    no_config=1
    echo "[build] No build configuration selected; building all non-ROS2 packages in repo."
    mapfile -t enabled_all < <(discover_all_nonros2_packages | sort -u)
    if [[ ${#enabled_all[@]} -eq 0 ]]; then
      echo "[build] ERROR: No buildable non-ROS2 packages found in repo." >&2
      return 1
    fi
  else
    has_jq || { echo "[build] ERROR: jq is required for target JSON parsing." >&2; return 1; }
    mapfile -t enabled_all < <(resolve_enabled_with_metadata)
    if [[ ${#enabled_all[@]} -eq 0 ]]; then
      echo "[build] ERROR: No enabled packages in configuration: ${config_file}" >&2
      return 1
    fi
  fi

  local build_set=()
  declare -A build_has=()
  for pkg_key in "${enabled_all[@]}"; do
    pkg_key_is_ros2 "${pkg_key}" && continue
    if [[ "${no_config}" == "1" ]]; then
      build_set+=("${pkg_key}")
      build_has["${pkg_key}"]=1
      continue
    fi

    if [[ "${pkg_key}" == components/* || "${pkg_key}" == middleware/* || "${pkg_key}" == application/native/* ]]; then
      build_set+=("${pkg_key}")
      build_has["${pkg_key}"]=1
    fi
  done

  if [[ ${#build_set[@]} -eq 0 ]]; then
    echo "[build] No non-ROS2 packages to build."
    return 0
  fi

  declare -A indeg=()
  declare -A outs=()

  for pkg in "${build_set[@]}"; do
    indeg["${pkg}"]=0
    outs["${pkg}"]=""
  done

  for pkg in "${build_set[@]}"; do
    while IFS= read -r dep; do
      [[ -z "${dep}" ]] && continue
      [[ -n "${build_has[${dep}]:-}" ]] || continue
      indeg["${pkg}"]=$((indeg["${pkg}"] + 1))
      outs["${dep}"]+="${pkg} "
    done < <(read_package_deps "${pkg}")
  done

  local ready=()
  for pkg in "${build_set[@]}"; do
    if [[ "${indeg[${pkg}]}" -eq 0 ]]; then
      ready+=("${pkg}")
    fi
  done

  local total=${#build_set[@]}
  local done_count=0
  local running=0
  declare -A pid_pkg=()

  echo "[build] Non-ROS2 packages: ${total} (parallel build workers: ${PARALLEL_JOBS}, install serialized)"

  _tail_pkg_log() {
    local pkg_key="$1"
    [[ -n "${pkg_key}" ]] || return 0
    local safe
    safe="$(echo "${pkg_key}" | tr '/ ' '__')"
    local log_file="${LOG_ROOT}/cmake/pkgs/${safe}.log"
    if [[ -f "${log_file}" ]]; then
      echo "[build] ---- Last $(_log_tail_lines) lines of log: ${log_file} ----" >&2
      tail -n "$(_log_tail_lines)" "${log_file}" >&2 || true
      echo "[build] ---- End log ----" >&2
    else
      echo "[build] (no log file found for ${pkg_key} at ${log_file})" >&2
    fi
  }

  _kill_all_running_jobs() {
    local p
    for p in "${!pid_pkg[@]}"; do
      kill "${p}" 2>/dev/null || true
    done
    # Reap whatever is left to avoid background output after exit.
    for p in "${!pid_pkg[@]}"; do
      wait "${p}" 2>/dev/null || true
      unset "pid_pkg[${p}]"
    done
    running=0
  }

  _reap_finished_nonblocking() {
    REAP_PROGRESSED=0
    REAP_FAILED_RC=0
    REAP_FAILED_PKG=""
    for p in "${!pid_pkg[@]}"; do
      if kill -0 "${p}" 2>/dev/null; then
        continue
      fi
      local rc=0
      if ! wait "${p}"; then
        rc=$?
      fi

      local finished_pkg="${pid_pkg[${p}]}"
      unset "pid_pkg[${p}]"
      running=$((running - 1))

      if [[ "${rc}" -ne 0 ]]; then
        REAP_FAILED_RC="${rc}"
        REAP_FAILED_PKG="${finished_pkg}"
        return 0
      fi

      done_count=$((done_count + 1))
      if [[ "${LOG_SHOW_DONE}" == "1" ]]; then
        echo "[build] End (${done_count}/${total}): ${finished_pkg}"
      fi

      local dependents="${outs[${finished_pkg}]}"
      for dep_pkg in ${dependents}; do
        indeg["${dep_pkg}"]=$((indeg["${dep_pkg}"] - 1))
        if [[ "${indeg[${dep_pkg}]}" -eq 0 ]]; then
          ready+=("${dep_pkg}")
        fi
      done

      REAP_PROGRESSED=1
    done

    return 0
  }

  while [[ "${done_count}" -lt "${total}" ]]; do
    _reap_finished_nonblocking
    if [[ "${REAP_FAILED_RC}" -ne 0 ]]; then
      echo "[build] ERROR: Package build failed: ${REAP_FAILED_PKG}" >&2
      _tail_pkg_log "${REAP_FAILED_PKG}"
      _kill_all_running_jobs
      return "${REAP_FAILED_RC}"
    fi
    if [[ "${REAP_PROGRESSED}" -eq 1 ]]; then
      continue
    fi

    while [[ ${#ready[@]} -gt 0 && "${running}" -lt "${PARALLEL_JOBS}" ]]; do
      local pkg="${ready[0]}"
      ready=("${ready[@]:1}")

      if [[ "${LOG_SHOW_ENTER}" == "1" ]]; then
        echo "[build] Start: ${pkg}"
      fi

      (
        set -euo pipefail
        export SROBOTIS_IN_SCHED=1
        build_one_nonros2_pkg "${pkg}" "${want_py_wheels}"
      ) &
      local pid=$!
      pid_pkg["${pid}"]="${pkg}"
      running=$((running + 1))
    done

    _reap_finished_nonblocking
    if [[ "${REAP_FAILED_RC}" -ne 0 ]]; then
      echo "[build] ERROR: Package build failed: ${REAP_FAILED_PKG}" >&2
      _tail_pkg_log "${REAP_FAILED_PKG}"
      _kill_all_running_jobs
      return "${REAP_FAILED_RC}"
    fi
    if [[ "${REAP_PROGRESSED}" -eq 1 ]]; then
      continue
    fi

    if [[ "${running}" -eq 0 ]]; then
      echo "[build] ERROR: Dependency deadlock or empty ready queue. Remaining packages:" >&2
      for pkg in "${build_set[@]}"; do
        if [[ "${indeg[${pkg}]}" -gt 0 ]]; then
          echo "  - ${pkg} (indeg=${indeg[${pkg}]})" >&2
        fi
      done
      return 1
    fi

    local finished_pid=""
    if ! wait -n -p finished_pid; then
      local failed_pkg="${pid_pkg[${finished_pid}]:-}"
      if [[ -n "${failed_pkg}" ]]; then
        echo "[build] ERROR: Package build failed: ${failed_pkg}" >&2
        _tail_pkg_log "${failed_pkg}"
      else
        echo "[build] ERROR: Package build failed (unknown pid=${finished_pid})" >&2
      fi
      _kill_all_running_jobs
      return 1
    fi

    if [[ -n "${pid_pkg[${finished_pid}]:-}" ]]; then
      local finished_pkg="${pid_pkg[${finished_pid}]}"
      unset "pid_pkg[${finished_pid}]"
      running=$((running - 1))
      done_count=$((done_count + 1))
      if [[ "${LOG_SHOW_DONE}" == "1" ]]; then
        echo "[build] End (${done_count}/${total}): ${finished_pkg}"
      fi

      local dependents="${outs[${finished_pkg}]}"
      for dep_pkg in ${dependents}; do
        indeg["${dep_pkg}"]=$((indeg["${dep_pkg}"] - 1))
        if [[ "${indeg[${dep_pkg}]}" -eq 0 ]]; then
          ready+=("${dep_pkg}")
        fi
      done
    fi
  done

  generate_components_cmake_package "${build_set[@]}"
  export SROBOTIS_NONROS2_BUILT=1
  export SROBOTIS_NONROS2_BUILT_CONFIG="${BUILD_CONFIG_FILE:-}"
  export SROBOTIS_NONROS2_BUILT_PREFIX="${PREFIX}"
  return 0
}

nonros2_deps_has_key() {
  local key="$1"
  shift
  local item
  for item in "$@"; do
    [[ "${item}" == "${key}" ]] && return 0
  done
  return 1
}

resolve_nonros2_dep_pkg_key() {
  local dep="$1"
  if [[ "${dep}" == */* ]]; then
    echo "${dep}"
    return 0
  fi

  local dep_dir
  dep_dir="$(pkg_key_to_dir "${dep}")"
  if [[ -d "${dep_dir}" ]]; then
    echo "${dep}"
    return 0
  fi

  local matches=()
  local candidate
  while IFS= read -r candidate; do
    [[ -n "${candidate}" ]] || continue
    [[ "${candidate##*/}" == "${dep}" ]] || continue
    matches+=("${candidate}")
  done < <(discover_all_nonros2_packages)

  if [[ ${#matches[@]} -eq 1 ]]; then
    echo "${matches[0]}"
    return 0
  fi

  echo "${dep}"
}

resolve_nonros2_dependency_closure() {
  local root_pkg="$1"
  local include_self="${2:-0}"

  [[ -n "${root_pkg}" ]] || { echo "[build] ERROR: Package key required" >&2; return 1; }

  local ordered=()
  local visiting=()
  local visited=()

  _nonros2_deps_visit_pkg() {
    local pkg="$1"

    if nonros2_deps_has_key "${pkg}" "${visited[@]}"; then
      return 0
    fi
    if nonros2_deps_has_key "${pkg}" "${visiting[@]}"; then
      echo "[build] ERROR: Dependency cycle detected at package: ${pkg}" >&2
      return 1
    fi

    visiting+=("${pkg}")

    local dep
    while IFS= read -r dep; do
      [[ -n "${dep}" ]] || continue
      dep="$(resolve_nonros2_dep_pkg_key "${dep}")"

      if pkg_key_is_ros2 "${dep}"; then
        echo "[build] ERROR: mm --with-deps/--deps only supports non-ROS2 SDK dependencies: ${pkg} -> ${dep}" >&2
        return 1
      fi

      local dep_dir
      dep_dir="$(pkg_key_to_dir "${dep}")"
      local dep_src_dir
      dep_src_dir="$(pkg_overlay_src_dir "${dep}")"
      if [[ ! -d "${dep_dir}" && ! -d "${dep_src_dir}" ]]; then
        echo "[build] ERROR: Dependency package not found: ${pkg} -> ${dep}" >&2
        return 1
      fi
      if [[ ! -f "${dep_src_dir}/CMakeLists.txt" && ! -x "${dep_src_dir}/build.sh" && ! -x "${dep_dir}/build.sh" ]]; then
        echo "[build] ERROR: Dependency is not buildable by non-ROS2 builder: ${pkg} -> ${dep}" >&2
        return 1
      fi

      _nonros2_deps_visit_pkg "${dep}" || return $?
    done < <(read_package_deps "${pkg}")

    local next_visiting=()
    local item
    for item in "${visiting[@]}"; do
      [[ "${item}" == "${pkg}" ]] && continue
      next_visiting+=("${item}")
    done
    visiting=("${next_visiting[@]}")

    visited+=("${pkg}")
    if [[ "${include_self}" == "1" || "${pkg}" != "${root_pkg}" ]]; then
      ordered+=("${pkg}")
    fi
  }

  _nonros2_deps_visit_pkg "${root_pkg}" || return $?

  printf '%s\n' "${ordered[@]}"
}

build_nonros2_package_deps() {
  local root_pkg="$1"
  local include_self="${2:-0}"
  local want_py_wheels="${3:-0}"

  [[ -n "${root_pkg}" ]] || { echo "[build] ERROR: Package key required" >&2; return 1; }

  local ordered=()
  local closure_output
  if ! closure_output="$(resolve_nonros2_dependency_closure "${root_pkg}" "${include_self}")"; then
    return 1
  fi
  if [[ -n "${closure_output}" ]]; then
    mapfile -t ordered <<< "${closure_output}"
  fi

  if [[ ${#ordered[@]} -eq 0 ]]; then
    echo "[build] No SDK dependencies to build for package: ${root_pkg}"
    return 0
  fi

  echo "[build] SDK dependency packages: ${#ordered[@]}"
  local pkg
  for pkg in "${ordered[@]}"; do
    build_one_nonros2_pkg "${pkg}" "${want_py_wheels}" || return $?
  done

  generate_components_cmake_package "${ordered[@]}"
}

discover_all_nonros2_packages() {
  # Discover buildable non-ROS2 packages.
  #
  # "Buildable" means: contains CMakeLists.txt OR an executable build.sh.
  #
  # Output: pkg_key, one per line.
  local root="${REPO_ROOT}"
  [[ -d "${root}" ]] || return 0

  # 1) Collect all candidate dirs that look buildable.
  # 2) Reduce to "package roots" by removing any dir whose ancestor is also a candidate.
  #    This avoids treating subdirectories like vision/src as separate packages when the
  #    real package root is vision/.
  local candidate_dirs=()
  mapfile -t candidate_dirs < <(
    find "${root}" \
      \( \
        -path "${REPO_ROOT}/output" -o -path "${REPO_ROOT}/output/*" -o \
        -path "${REPO_ROOT}/.git" -o -path "${REPO_ROOT}/.git/*" -o \
        -path "${REPO_ROOT}/build" -o -path "${REPO_ROOT}/build/*" -o \
        -path "${REPO_ROOT}/tools" -o -path "${REPO_ROOT}/tools/*" -o \
        -path "${REPO_ROOT}/scripts" -o -path "${REPO_ROOT}/scripts/*" -o \
        -path "${REPO_ROOT}/target" -o -path "${REPO_ROOT}/target/*" -o \
        -path "*/node_modules" -o -path "*/node_modules/*" -o \
        -path "*/.venv" -o -path "*/.venv/*" -o \
        -path "*/__pycache__" -o -path "*/__pycache__/*" \
      \) -prune -o \
      -type f \( -name "CMakeLists.txt" -o -name "build.sh" \) \
      -print 2>/dev/null | sed 's#/[^/]*$##' | sort -u
  )

  if [[ ${#candidate_dirs[@]} -eq 0 ]]; then
    return 0
  fi

  declare -A cand_has=()
  local dir
  for dir in "${candidate_dirs[@]}"; do
    [[ -n "${dir}" ]] || continue
    cand_has["${dir}"]=1
  done

  for dir in "${candidate_dirs[@]}"; do
    [[ -n "${dir}" ]] || continue

    # Skip ROS2 workspaces/packages here; ROS2 is handled by colcon.
    local pkg_key="${dir#"${REPO_ROOT}"/}"
    if [[ "${pkg_key}" == middleware/ros2/* || "${pkg_key}" == application/ros2/* ]]; then
      continue
    fi

    local p="${dir%/*}"
    local is_nested=0
    while [[ -n "${p}" && "${p}" != "${dir}" ]]; do
      if [[ -n "${cand_has[${p}]:-}" ]]; then
        is_nested=1
        break
      fi
      # Stop once we reached repo root or filesystem root.
      if [[ "${p}" == "${REPO_ROOT}" || "${p}" == "/" ]]; then
        break
      fi
      p="${p%/*}"
    done

    [[ "${is_nested}" == "1" ]] && continue
    echo "${pkg_key}"
  done
}

generate_components_cmake_package() {
  local pkgs=("$@")
  local cmake_dir="${PREFIX}/lib/cmake/Components"
  mkdir -p "${cmake_dir}"

  local cfg="${cmake_dir}/ComponentsConfig.cmake"
  cat > "${cfg}" <<'EOF'
# Auto-generated by build/nonros2.sh (per-package build system)
#
# Provides imported targets under namespace `components::` for SDK consumers.
#
# Prefix layout:
#   <prefix>/include
#   <prefix>/lib
#   <prefix>/lib/cmake/Components/ComponentsConfig.cmake  (this file)

get_filename_component(_SROBOTIS_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

include(CMakeFindDependencyMacro)
find_dependency(Threads)

function(_srobotis_components_add_imported _tgt _libpath)
  if(TARGET "components::${_tgt}")
    return()
  endif()
  if(NOT EXISTS "${_libpath}")
    message(FATAL_ERROR "Components: missing library for target components::${_tgt}: ${_libpath}")
  endif()
  add_library("components::${_tgt}" UNKNOWN IMPORTED)
  set_target_properties("components::${_tgt}" PROPERTIES
    IMPORTED_LOCATION "${_libpath}"
    INTERFACE_INCLUDE_DIRECTORIES "${_SROBOTIS_PREFIX}/include"
  )
endfunction()

function(_srobotis_components_add_header_only _tgt)
  if(TARGET "components::${_tgt}")
    return()
  endif()
  add_library("components::${_tgt}" INTERFACE IMPORTED)
  set_target_properties("components::${_tgt}" PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_SROBOTIS_PREFIX}/include"
  )
endfunction()

EOF

  declare -A added=()
  local pkg
  for pkg in "${pkgs[@]}"; do
    [[ "${pkg}" == components/* ]] || continue
    [[ "${pkg}" == components/thirdparty/* ]] && continue

    local bdir
    bdir="$(pkg_build_dir "${pkg}")"
    local manifest="${bdir}/install_manifest.txt"
    [[ -f "${manifest}" ]] || continue

    local libs=()
    while IFS= read -r line; do
      [[ "${line}" == "${PREFIX}/lib/lib"*".so"* || "${line}" == "${PREFIX}/lib/lib"*".a" ]] || continue
      libs+=("${line}")
    done < "${manifest}"

    if [[ ${#libs[@]} -gt 0 ]]; then
      local libpath="${libs[0]}"
      local base
      base="$(basename "${libpath}")"
      local name="${base#lib}"
      name="${name%%.so*}"
      name="${name%%.a}"
      added["${name}"]=1
      printf ' _srobotis_components_add_imported("%s" "%s")\n' "${name}" "${libpath}" >> "${cfg}"
    else
      local leaf="${pkg##*/}"
      added["${leaf}"]=1
      printf ' _srobotis_components_add_header_only("%s")\n' "${leaf}" >> "${cfg}"
    fi
  done

  # Supplement: export any lib*.so / lib*.a in PREFIX/lib that were not already added
  # (e.g. from a previous build so motion can find components::mlink_device even if this run only built 3 pkgs)
  local lib_dir="${PREFIX}/lib"
  if [[ -d "${lib_dir}" ]]; then
    local f
    for f in "${lib_dir}"/lib*.so "${lib_dir}"/lib*.a; do
      [[ -f "${f}" ]] || continue
      local base
      base="$(basename "${f}")"
      local name="${base#lib}"
      name="${name%%.so*}"
      name="${name%%.a}"
      [[ -n "${name}" ]] || continue
      [[ -z "${added[${name}]:-}" ]] || continue
      added["${name}"]=1
      printf ' _srobotis_components_add_imported("%s" "%s")\n' "${name}" "${f}" >> "${cfg}"
    done
  fi
}
