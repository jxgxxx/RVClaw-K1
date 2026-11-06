#!/usr/bin/env bash
#
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#

# Common helpers for the SDK build system.
#
# This file is meant to be SOURCED by build scripts (e.g. build/build.sh).
#
# Responsibilities:
# - repo/prefix defaulting (best-effort)
# - target config loading
# - package metadata access (delegates jq parts to build/jq_utils.sh)
# - ROS2 package discovery helpers
# - system dependency check/install

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[build] ERROR: build/common.sh must be sourced, not executed." >&2
  exit 1
fi

# Best-effort defaults if caller didn't define them yet.
if [[ -z "${REPO_ROOT:-}" ]]; then
  REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi
: "${OUTPUT_ROOT:=${REPO_ROOT}/output}"
: "${LOG_ROOT:=${OUTPUT_ROOT}/log}"
: "${CM_BUILD_ROOT:=${OUTPUT_ROOT}/build/cmake}"
: "${ROS2_BUILD_ROOT:=${OUTPUT_ROOT}/build/ros2}"
: "${ROS2_LOG_ROOT:=${OUTPUT_ROOT}/log/ros2}"
: "${STAGING_PREFIX:=${OUTPUT_ROOT}/staging}"
: "${ROOTFS_PREFIX:=${OUTPUT_ROOT}/rootfs}"
: "${PREFIX:=${STAGING_PREFIX}}"

# Logging
# LOG_LEVEL: quiet|normal|verbose (default: quiet)
: "${LOG_LEVEL:=${LOG_LEVEL:-quiet}}"
: "${LOG_TEE:=${LOG_TEE:-1}}"
: "${LOG_TAIL_LINES:=${LOG_TAIL_LINES:-200}}"

# Console status lines control:
# - LOG_SHOW_ENTER controls per-package "Start: <pkg>" lines.
# Default: on (so you always know what is being built).
if [[ -z "${LOG_SHOW_ENTER:-}" ]]; then
  LOG_SHOW_ENTER=1
fi
: "${LOG_SHOW_DONE:=${LOG_SHOW_DONE:-1}}"

# jq helpers
source "${REPO_ROOT}/build/jq_utils.sh"

# ----------------------------------------------------------------------------
# Logging / verbosity control
# ----------------------------------------------------------------------------

_log_tail_lines() {
  echo "${LOG_TAIL_LINES}"
}

_log_is_verbose() {
  [[ "${LOG_LEVEL}" == "verbose" ]]
}

_log_should_capture() {
  [[ "${LOG_LEVEL}" == "quiet" || "${LOG_LEVEL}" == "normal" ]]
}

run_logged_overwrite() {
  # Usage: run_logged_overwrite <logfile> <cmd> [args...]
  local logfile="$1"
  shift
  mkdir -p "$(dirname "${logfile}")"

  if _log_is_verbose; then
    if [[ "${LOG_TEE}" == "1" ]]; then
      # shellcheck disable=SC2129
      { echo ">>> $*"; "$@"; } 2>&1 | tee "${logfile}"
      local rc=${PIPESTATUS[0]}
      return "${rc}"
    fi
    "$@"
    return $?
  fi

  { echo ">>> $*"; "$@"; } >"${logfile}" 2>&1
  local rc=$?
  if [[ "${rc}" -ne 0 ]]; then
    echo "[build] ERROR: command failed (rc=${rc}). See log: ${logfile}" >&2
    tail -n "$(_log_tail_lines)" "${logfile}" >&2 || true
    return "${rc}"
  fi
  return 0
}

run_logged_append() {
  # Usage: run_logged_append <logfile> <cmd> [args...]
  local logfile="$1"
  shift
  mkdir -p "$(dirname "${logfile}")"

  if _log_is_verbose; then
    if [[ "${LOG_TEE}" == "1" ]]; then
      { echo ">>> $*"; "$@"; } 2>&1 | tee -a "${logfile}"
      local rc=${PIPESTATUS[0]}
      return "${rc}"
    fi
    "$@"
    return $?
  fi

  { echo ">>> $*"; "$@"; } >>"${logfile}" 2>&1
  local rc=$?
  if [[ "${rc}" -ne 0 ]]; then
    echo "[build] ERROR: command failed (rc=${rc}). See log: ${logfile}" >&2
    tail -n "$(_log_tail_lines)" "${logfile}" >&2 || true
    return "${rc}"
  fi
  return 0
}

# Return the on-disk package.json path for a package key like:
# - components/peripherals/motor
# - middleware/ros2/mlink/cpp
# - application/ros2/lekiwi/motion
package_json_path() {
  local pkg_key="$1"
  echo "${REPO_ROOT}/${pkg_key}/package.json"
}

# Return the on-disk package.xml path for a package key.
# For components/thirdparty/<name>, use platform overlay dir so package.xml lives under platform.
package_xml_path() {
  local pkg_key="$1"
  if [[ "${pkg_key}" == components/thirdparty/* ]]; then
    local name="${pkg_key#components/thirdparty/}"
    echo "${REPO_ROOT}/platform/generic/components/thirdparty/${name}/package.xml"
  else
    echo "${REPO_ROOT}/${pkg_key}/package.xml"
  fi
}

# Resolve package name (from <name> in package.xml) to pkg_key. Outputs pkg_key or empty.
# Scans package.xml under components/ and platform/generic/components/thirdparty/.
get_pkg_key_by_package_name() {
  local name="$1"
  [[ -n "${name}" ]] || return 0
  local f
  while IFS= read -r f; do
    [[ -f "${f}" ]] || continue
    local dir="${f%/*}"
    local pkg_key
    if [[ "${dir}" == *"/platform/generic/components/thirdparty/"* ]]; then
      pkg_key="components/thirdparty/$(basename "${dir}")"
    else
      pkg_key="${dir#"${REPO_ROOT}"/}"
    fi
    local xml_name
    xml_name="$(sed -n 's/.*<name> *\([^<]*\) *<\/name>.*/\1/p' "${f}" 2>/dev/null | head -n 1 | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
    if [[ "${xml_name}" == "${name}" ]]; then
      echo "${pkg_key}"
      return 0
    fi
  done < <(find "${REPO_ROOT}/components" "${REPO_ROOT}/platform/generic/components/thirdparty" -name "package.xml" -type f 2>/dev/null)
  return 0
}

# Read <depend> from package.xml. Output one pkg_key per line.
# <depend> content is package name (from other package.xml <name>); resolved to pkg_key. If content contains "/", treated as pkg_key.
# Optional attribute arch="..." limits the dependency to matching SDK_BUILD_ARCH/uname -m.
read_package_xml_deps() {
  local pkg_key="$1"
  local pkg_xml
  pkg_xml="$(package_xml_path "${pkg_key}")"
  [[ -f "${pkg_xml}" ]] || return 0

  while IFS= read -r line; do
    [[ -z "${line}" ]] && continue
    local dep arch_filter
    dep="$(echo "${line}" | sed -n 's/.*<depend[^>]*> *\([^<]*\) *<\/depend>.*/\1/p' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
    [[ -n "${dep}" ]] || continue
    arch_filter="$(xml_line_attr "${line}" "arch")"
    xml_arch_matches_current "${arch_filter}" || continue

    if [[ "${dep}" == */* ]]; then
      echo "${dep}"
    else
      local resolved
      resolved="$(get_pkg_key_by_package_name "${dep}")"
      if [[ -n "${resolved}" ]]; then
        echo "${resolved}"
      else
        echo "${dep}"
      fi
    fi
  done < <(grep -E '<depend([[:space:]>])' "${pkg_xml}" 2>/dev/null)
}

# Read ROS2 package dependencies that map to SDK non-ROS2 components.
# This keeps ROS/system dependencies (rclcpp, sensor_msgs, etc.) in colcon/ROS,
# while allowing mm --with-deps/--deps to prebuild SDK underlay components.
read_ros2_sdk_nonros2_deps() {
  local pkg_key="$1"
  local pkg_xml
  pkg_xml="$(package_xml_path "${pkg_key}")"
  [[ -f "${pkg_xml}" ]] || return 0

  local emitted=()

  while IFS= read -r line; do
    [[ -z "${line}" ]] && continue

    local dep arch_filter resolved dep_xml dep_dir build_type is_buildable
    dep="$(echo "${line}" | sed -n 's/.*<\(depend\|build_depend\|build_export_depend\|exec_depend\)[^>]*> *\([^<]*\) *<\/\1>.*/\2/p' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
    [[ -n "${dep}" ]] || continue

    arch_filter="$(xml_line_attr "${line}" "arch")"
    xml_arch_matches_current "${arch_filter}" || continue

    if [[ "${dep}" == */* ]]; then
      resolved="${dep}"
    else
      resolved="$(get_pkg_key_by_package_name "${dep}")"
    fi
    [[ -n "${resolved}" ]] || continue

    # Current SDK underlay packages live under components/. Ignore ROS/system
    # packages and any package name that only colcon should resolve.
    [[ "${resolved}" == components/* ]] || continue

    dep_xml="$(package_xml_path "${resolved}")"
    if [[ -f "${dep_xml}" ]]; then
      build_type="$(sed -n 's/.*<build_type>\([^<]*\)<\/build_type>.*/\1/p' "${dep_xml}" 2>/dev/null | head -n 1)"
      [[ "${build_type}" == "ament_cmake" || "${build_type}" == "ament_python" ]] && continue
      grep -Eq '<buildtool_depend> *(ament_|rosidl_)' "${dep_xml}" 2>/dev/null && continue
    fi

    dep_dir="${REPO_ROOT}/${resolved}"
    is_buildable=0
    if [[ -f "${dep_dir}/CMakeLists.txt" || -x "${dep_dir}/build.sh" ]]; then
      is_buildable=1
    elif [[ "${resolved}" == components/thirdparty/* ]]; then
      local thirdparty_name overlay_dir
      thirdparty_name="${resolved#components/thirdparty/}"
      overlay_dir="${REPO_ROOT}/platform/generic/components/thirdparty/${thirdparty_name}"
      [[ -f "${overlay_dir}/CMakeLists.txt" || -x "${overlay_dir}/build.sh" ]] && is_buildable=1
    fi
    [[ "${is_buildable}" == "1" ]] || continue

    local already_emitted=0
    local emitted_dep
    for emitted_dep in "${emitted[@]}"; do
      if [[ "${emitted_dep}" == "${resolved}" ]]; then
        already_emitted=1
        break
      fi
    done
    [[ "${already_emitted}" == "1" ]] && continue

    emitted+=("${resolved}")
    echo "${resolved}"
  done < <(grep -E '<(depend|build_depend|build_export_depend|exec_depend)([[:space:]>])' "${pkg_xml}" 2>/dev/null)
}

# Normalize architecture names used by package.xml platform filters.
normalize_build_arch() {
  local arch="$1"
  case "${arch}" in
    x86_64|amd64|AMD64)
      echo "x86_64"
      ;;
    riscv64|rv64|riscv)
      echo "riscv64"
      ;;
    *)
      echo "${arch}"
      ;;
  esac
}

get_build_arch() {
  normalize_build_arch "${SDK_BUILD_ARCH:-$(uname -m)}"
}

xml_line_attr() {
  local line="$1"
  local attr="$2"
  echo "${line}" | sed -n "s/.*${attr}=\"\\([^\"]*\\)\".*/\\1/p"
}

xml_arch_matches_current() {
  local arch_filter="$1"
  [[ -z "${arch_filter}" ]] && return 0

  local current_arch
  current_arch="$(get_build_arch)"

  local item
  arch_filter="${arch_filter//,/ }"
  for item in ${arch_filter}; do
    [[ "$(normalize_build_arch "${item}")" == "${current_arch}" ]] && return 0
  done

  return 1
}

normalize_board_family() {
  local board="$1"
  case "${board}" in
    k1*|K1*)
      echo "k1"
      ;;
    k3*|K3*)
      echo "k3"
      ;;
    *)
      echo "${board}"
      ;;
  esac
}

target_board_family() {
  local config_file="${BUILD_CONFIG_FILE:-}"
  [[ -z "${config_file}" || ! -f "${config_file}" ]] && return 0
  has_jq || return 0
  normalize_board_family "$(jq -r '.board // ""' "${config_file}" 2>/dev/null || true)"
}

xml_board_matches_current() {
  local board_filter="$1"
  [[ -z "${board_filter}" ]] && return 0

  local current_board
  current_board="$(target_board_family)"
  [[ -z "${current_board}" ]] && return 0

  local item
  board_filter="${board_filter//,/ }"
  for item in ${board_filter}; do
    [[ "$(normalize_board_family "${item}")" == "${current_board}" ]] && return 0
  done

  return 1
}

deps_when_matches_current() {
  local when="${1:-all}"
  case "${when:-all}" in
    all)
      return 0
      ;;
    native)
      [[ "${SROBOTIS_IN_DOCKER_BUILD:-0}" != "1" && "${SROBOTIS_CROSS_BUILD:-0}" != "1" ]]
      return $?
      ;;
    docker)
      [[ "${SROBOTIS_IN_DOCKER_BUILD:-0}" == "1" ]]
      return $?
      ;;
    cross)
      [[ "${SROBOTIS_CROSS_BUILD:-0}" == "1" ]]
      return $?
      ;;
    *)
      echo "[deps] ERROR: Invalid when='${when}'" >&2
      return 2
      ;;
  esac
}

deps_shell_quote() {
  local quoted
  printf -v quoted '%q' "$1"
  printf '%s\n' "${quoted}"
}

deps_check_cmd_from_kind() {
  local dep_name="$1"
  local check_kind="${2:-}"
  local check_arg="${3:-}"
  local legacy_check="${4:-}"
  local quoted

  if [[ -n "${legacy_check}" && -z "${check_kind}" ]]; then
    printf '%s\n' "${legacy_check}"
    return 0
  fi

  case "${check_kind:-dpkg}" in
    dpkg)
      quoted="$(deps_shell_quote "${check_arg:-${dep_name}}")"
      # shellcheck disable=SC2016
      printf 'test "$(dpkg-query -W -f=\${Status} %s 2>/dev/null || true)" = "install ok installed"\n' "${quoted}"
      ;;
    command)
      quoted="$(deps_shell_quote "${check_arg:-${dep_name}}")"
      printf 'command -v %s\n' "${quoted}"
      ;;
    pkg-config)
      quoted="$(deps_shell_quote "${check_arg:-${dep_name}}")"
      printf 'pkg-config --exists %s\n' "${quoted}"
      ;;
    file)
      quoted="$(deps_shell_quote "${check_arg}")"
      printf 'test -e %s\n' "${quoted}"
      ;;
    rustlib)
      quoted="$(deps_shell_quote "${check_arg:-riscv64*-unknown-linux-gnu}")"
      printf 'find /usr/lib -path "*/rustlib/%s/lib/libcore-*.rlib" -print -quit | grep -q .\n' "${quoted}"
      ;;
    *)
      echo "[deps] ERROR: Invalid check_kind='${check_kind}' for ${dep_name}" >&2
      return 1
      ;;
  esac
}

deps_required_type_from_line() {
  local line="$1"
  local required
  required="$(xml_line_attr "${line}" "required")"
  case "${required:-required}" in
    required|true|yes|1)
      echo "required"
      ;;
    optional|false|no|0)
      echo "optional"
      ;;
    *)
      echo "[deps] ERROR: Invalid required='${required}'" >&2
      return 1
      ;;
  esac
}

deps_validate_system_depend_metadata() {
  local dep_name="$1"
  local arch_filter="$2"
  local realm="$3"
  local when="$4"
  local check_kind="$5"

  if [[ "${arch_filter}" == "cross" ]]; then
    echo "[deps] ERROR: arch='cross' is not allowed for ${dep_name}; use when='cross' instead" >&2
    return 1
  fi

  case "${realm:-}" in
    ""|host|target|both|skip)
      ;;
    *)
      echo "[deps] ERROR: Invalid realm='${realm}' for ${dep_name}" >&2
      return 1
      ;;
  esac

  case "${when:-all}" in
    all|native|docker|cross)
      ;;
    *)
      echo "[deps] ERROR: Invalid when='${when}' for ${dep_name}" >&2
      return 1
      ;;
  esac

  case "${check_kind:-}" in
    ""|dpkg|command|pkg-config|file|rustlib)
      ;;
    *)
      echo "[deps] ERROR: Invalid check_kind='${check_kind}' for ${dep_name}" >&2
      return 1
      ;;
  esac
}

deps_option_matches_current() {
  local pkg_key="$1"
  local option_key="${2:-}"
  local option_value="${3:-}"
  [[ -z "${option_key}" ]] && return 0
  [[ -z "${pkg_key}" ]] && return 0

  local config_file="${BUILD_CONFIG_FILE:-}"
  if [[ -z "${config_file}" || ! -f "${config_file}" ]] || ! has_jq; then
    return 0
  fi

  local selected=()
  local item wanted
  mapfile -t selected < <(get_target_option_list "${pkg_key}" ".${option_key}[]? // empty")
  [[ ${#selected[@]} -eq 0 ]] && return 1

  option_value="${option_value//,/ }"
  option_value="${option_value//;/ }"
  for item in "${selected[@]}"; do
    for wanted in ${option_value}; do
      [[ "${item}" == "${wanted}" ]] && return 0
    done
  done
  return 1
}

# Read <system_depend> from package.xml. Output lines: required|dep_name|check_cmd
# Default check_cmd verifies dpkg status is "install ok installed". Optional attribute check="..." overrides it.
# Optional attribute arch="..." limits the dependency to matching SDK_BUILD_ARCH/uname -m.
read_sysdeps_lines_from_xml_file() {
  local pkg_xml="$1"
  local pkg_key="${2:-}"
  [[ -f "${pkg_xml}" ]] || return 0
  while IFS= read -r line; do
    [[ -z "${line}" ]] && continue
    local name check_cmd arch_filter board_filter realm when check_kind check_arg option_key option_value dep_type
    # Extract name from PCDATA before </system_depend> (avoid matching ">" inside check="...2>/dev/null...")
    name="$(echo "${line}" | sed -n 's/.*> *\([^<]*\) *<\/system_depend>.*/\1/p' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
    check_cmd="$(xml_line_attr "${line}" "check")"
    arch_filter="$(xml_line_attr "${line}" "arch")"
    board_filter="$(xml_line_attr "${line}" "board")"
    realm="$(xml_line_attr "${line}" "realm")"
    when="$(xml_line_attr "${line}" "when")"
    check_kind="$(xml_line_attr "${line}" "check_kind")"
    check_arg="$(xml_line_attr "${line}" "check_arg")"
    option_key="$(xml_line_attr "${line}" "option_key")"
    option_value="$(xml_line_attr "${line}" "option_value")"
    [[ -z "${name}" ]] && continue
    deps_validate_system_depend_metadata "${name}" "${arch_filter}" "${realm}" "${when}" "${check_kind}" || return 1
    [[ "${realm}" == "skip" ]] && continue
    deps_when_matches_current "${when:-all}" || continue
    xml_arch_matches_current "${arch_filter}" || continue
    xml_board_matches_current "${board_filter}" || continue
    deps_option_matches_current "${pkg_key}" "${option_key}" "${option_value}" || continue
    dep_type="$(deps_required_type_from_line "${line}")" || return 1
    check_cmd="$(deps_check_cmd_from_kind "${name}" "${check_kind}" "${check_arg}" "${check_cmd}")" || return 1
    echo "${dep_type}|${name}|${check_cmd}"
  done < <(grep -E '<system_depend' "${pkg_xml}" 2>/dev/null)
}

read_package_xml_sysdeps_lines() {
  local pkg_key="$1"
  local pkg_xml
  pkg_xml="$(package_xml_path "${pkg_key}")"
  read_sysdeps_lines_from_xml_file "${pkg_xml}" "${pkg_key}"
}

# Read dependencies for a package key:
# 1) package.xml <depend> if present
# 2) package.json
read_package_deps() {
  local pkg_key="$1"
  local pkg_xml
  pkg_xml="$(package_xml_path "${pkg_key}")"
  if [[ -f "${pkg_xml}" ]]; then
    local deps
    deps="$(read_package_xml_deps "${pkg_key}")"
    if [[ -n "${deps}" ]]; then
      echo "${deps}"
      return 0
    fi
  fi
  local pj
  pj="$(package_json_path "${pkg_key}")"
  read_package_json_deps "${pj}"
}

# Read system dependency lines (required|name|check or optional|name|check):
# 1) package.xml <system_depend> if present
# 2) package.json
read_package_sysdeps_lines() {
  local pkg_key="$1"
  local pkg_xml
  pkg_xml="$(package_xml_path "${pkg_key}")"
  if [[ -f "${pkg_xml}" ]] && grep -q '<system_depend' "${pkg_xml}" 2>/dev/null; then
    read_package_xml_sysdeps_lines "${pkg_key}"
    return 0
  fi
  local pj
  pj="$(package_json_path "${pkg_key}")"
  read_package_json_sysdeps_lines "${pj}"
}

# Return 0 if current BUILD_CONFIG_FILE enables any ROS2 packages.
target_needs_ros2() {
  local config_file="${BUILD_CONFIG_FILE:-}"
  [[ -z "${config_file}" || ! -f "${config_file}" ]] && return 1
  has_jq || return 1

  # enabled_packages: ["middleware/ros2/...", "application/ros2/...", ...]
  if jq -e '.enabled_packages[]? | startswith("middleware/ros2/") or startswith("application/ros2/")' \
    "${config_file}" >/dev/null 2>&1; then
    return 0
  fi
  return 1
}

# Load build configuration from JSON file
load_build_config() {
  local config_file="${BUILD_TARGET_FILE:-}"

  # If BUILD_TARGET is set but BUILD_TARGET_FILE is not, construct the path
  if [[ -n "${BUILD_TARGET:-}" && -z "${config_file}" ]]; then
    config_file="${REPO_ROOT}/target/${BUILD_TARGET}.json"
  fi

  # Only accept target configs from the current repo to avoid accidentally
  # using BUILD_TARGET_FILE exported by another workspace.
  if [[ -n "${config_file}" ]]; then
    local abs_cfg=""
    abs_cfg="$(cd "$(dirname "${config_file}")" 2>/dev/null && pwd)/$(basename "${config_file}")" 2>/dev/null || true
    if [[ -n "${abs_cfg}" ]]; then
      config_file="${abs_cfg}"
    fi
    if [[ "${config_file}" != "${REPO_ROOT}/target/"* ]]; then
      local repo_config_file=""
      if [[ -n "${BUILD_TARGET:-}" && -f "${REPO_ROOT}/target/${BUILD_TARGET}.json" ]]; then
        repo_config_file="${REPO_ROOT}/target/${BUILD_TARGET}.json"
      fi
      if [[ -n "${repo_config_file}" ]]; then
        config_file="${repo_config_file}"
      else
        echo "[build] Ignoring external BUILD_TARGET_FILE (not under ${REPO_ROOT}/target): ${config_file}" >&2
        config_file=""
      fi
    fi
  fi

  # If no config file specified, return (use default behavior)
  if [[ -z "${config_file}" || ! -f "${config_file}" ]]; then
    return 0
  fi

  # Export config file path for use by other functions
  export BUILD_CONFIG_FILE="${config_file}"
  export BUILD_TARGET_FILE="${config_file}"
  if [[ -z "${BUILD_TARGET:-}" ]]; then
    BUILD_TARGET="$(basename "${config_file}" .json)"
    export BUILD_TARGET
  fi

  # Load options from config
  local parallel_jobs_config
  parallel_jobs_config="$(target_parallel_jobs || true)"
  if [[ -n "${parallel_jobs_config}" && "${SROBOTIS_PARALLEL_JOBS_EXPLICIT:-0}" != "1" ]]; then
    export PARALLEL_JOBS="${parallel_jobs_config}"
  fi

  echo "[build] Using configuration: ${config_file}"
}

# Get ROS2 package name from package.xml in a directory
get_ros2_package_name() {
  local pkg_dir="$1"
  if [[ -f "${pkg_dir}/package.xml" ]]; then
    grep '<name>' "${pkg_dir}/package.xml" | sed 's/.*<name>\([^<]*\)<\/name>.*/\1/' | head -n 1
  else
    # Fallback: use directory name
    basename "${pkg_dir}"
  fi
}

# Read <export><build_type> from package.xml. Used to decide ROS2 vs CMake build.
# Outputs: ament_cmake, ament_python, cmake, or empty (treated as cmake).
get_package_build_type() {
  local pkg_dir="$1"
  local xml="${pkg_dir}/package.xml"
  [[ -f "${xml}" ]] || return 0
  sed -n 's/.*<build_type>\([^<]*\)<\/build_type>.*/\1/p' "${xml}" | head -n 1
}

# Find all ROS2 packages in a directory (recursive)
find_ros2_packages_in_dir() {
  local dir="$1"
  local packages=()

  if [[ ! -d "${dir}" ]]; then
    return 1
  fi

  # Find all package.xml files recursively
  while IFS= read -r -d '' pkg_xml; do
    local pkg_dir="${pkg_xml%/*}"
    local pkg_name
    pkg_name="$(get_ros2_package_name "${pkg_dir}")"
    [[ -n "${pkg_name}" ]] && packages+=("${pkg_name}")
  done < <(find "${dir}" -name "package.xml" -type f -print0 2>/dev/null)

  # Output all found packages
  printf '%s\n' "${packages[@]}"
}

# Map config package path to actual ROS2 package name(s)
map_config_path_to_ros2_package() {
  local category="$1"  # e.g., "application/ros2"
  local config_path="$2"  # e.g., "lekiwi/motion"
  local pkg_dir="${REPO_ROOT}/${category}/${config_path}"

  # Check if this is a ROS2 package (has package.xml)
  if [[ -f "${pkg_dir}/package.xml" ]]; then
    get_ros2_package_name "${pkg_dir}"
    return 0
  fi

  # If directory exists but no package.xml, it might be a directory with sub-packages
  # Try to find all ROS2 packages in this directory
  if [[ -d "${pkg_dir}" ]]; then
    local sub_packages
    mapfile -t sub_packages < <(find_ros2_packages_in_dir "${pkg_dir}")
    if [[ ${#sub_packages[@]} -gt 0 ]]; then
      printf '%s\n' "${sub_packages[@]}"
      return 0
    fi
  fi

  return 1
}

# ============================================================================
# System dependency checking and installation
# ============================================================================

check_single_dependency() {
  local dep_name="$1"
  local check_cmd="$2"
  local debug="${DEBUG_DEPS:-0}"

  if [[ "${debug}" == "1" ]]; then
    echo "[deps] Checking ${dep_name}: ${check_cmd}" >&2
  fi

  if eval "${check_cmd}" >/dev/null 2>&1; then
    return 0
  else
    return 1
  fi
}

collect_system_dependencies() {
  local config_file="${BUILD_CONFIG_FILE:-}"

  # Always include build base deps (build/package.xml) so m/mm check build tools and repo.
  if [[ -f "${REPO_ROOT}/build/package.xml" ]] && grep -q '<system_depend' "${REPO_ROOT}/build/package.xml" 2>/dev/null; then
    while IFS='|' read -r dep_type dep_name check_cmd; do
      [[ -z "${dep_type}" || -z "${dep_name}" || -z "${check_cmd}" ]] && continue
      echo "build|${dep_type}|${dep_name}|${check_cmd}"
    done < <(read_package_sysdeps_lines "build")
  fi

  # Include ROS2 deps only when the current build needs ROS2.
  # - build.sh sets BUILD_NEEDS_ROS2=1 for "ros2" and "all"
  # - build.sh package path handles per-package detection separately
  if [[ "${BUILD_NEEDS_ROS2:-0}" == "1" ]]; then
    local ros2_deps_xml="${REPO_ROOT}/build/package_ros2.xml"
    if [[ -f "${ros2_deps_xml}" ]] && grep -q '<system_depend' "${ros2_deps_xml}" 2>/dev/null; then
      while IFS='|' read -r dep_type dep_name check_cmd; do
        [[ -z "${dep_type}" || -z "${dep_name}" || -z "${check_cmd}" ]] && continue
        echo "build_ros2_deps|${dep_type}|${dep_name}|${check_cmd}"
      done < <(read_sysdeps_lines_from_xml_file "${ros2_deps_xml}")
    fi
  fi

  local enabled_all=()
  if [[ -n "${config_file}" && -f "${config_file}" ]]; then
    if ! has_jq; then
      echo "[deps] WARNING: jq not available, skipping target package dependency check" >&2
      return 0
    fi
    mapfile -t enabled_all < <(resolve_enabled_with_metadata)
  elif [[ "${BUILD_NEEDS_NONROS2:-0}" == "1" ]] && command -v discover_all_nonros2_packages >/dev/null 2>&1; then
    # No target selected: match build_nonros2_enabled_packages(), which builds
    # every discovered non-ROS2 package for all/cmake commands in this mode.
    mapfile -t enabled_all < <(discover_all_nonros2_packages | sort -u)
  else
    return 0
  fi

  for pkg_path in "${enabled_all[@]}"; do
    while IFS='|' read -r dep_type dep_name check_cmd; do
      [[ -z "${dep_type}" || -z "${dep_name}" || -z "${check_cmd}" ]] && continue
      echo "${pkg_path}|${dep_type}|${dep_name}|${check_cmd}"
    done < <(read_package_sysdeps_lines "${pkg_path}")
  done
}

check_system_dependencies() {
  local config_file="${BUILD_CONFIG_FILE:-}"
  local missing_required=()
  local missing_optional=()
  local checked_packages=()

  echo "[deps] Checking system dependencies..."

  local deps_lines
  mapfile -t deps_lines < <(collect_system_dependencies)

  if [[ ${#deps_lines[@]} -eq 0 ]]; then
    echo "[deps] No system dependencies defined in configuration"
    return 0
  fi

  for dep_line in "${deps_lines[@]}"; do
    IFS='|' read -r _pkg_path dep_type dep_name check_cmd <<< "${dep_line}"

    local already_checked=false
    for checked in "${checked_packages[@]}"; do
      if [[ "${checked}" == "${dep_name}" ]]; then
        already_checked=true
        break
      fi
    done
    [[ "${already_checked}" == "true" ]] && continue

    if check_single_dependency "${dep_name}" "${check_cmd}"; then
      echo "[deps] ✓ ${dep_name}: found"
      checked_packages+=("${dep_name}")
    else
      echo "[deps] ✗ ${dep_name}: missing (${dep_type})"
      checked_packages+=("${dep_name}")
      if [[ "${dep_type}" == "required" ]]; then
        missing_required+=("${dep_name}")
      else
        missing_optional+=("${dep_name}")
      fi
    fi
  done

  if [[ ${#missing_required[@]} -gt 0 ]]; then
    export MISSING_REQUIRED_DEPS="${missing_required[*]}"
    echo "[deps] Missing required dependencies: ${missing_required[*]}"
  else
    export MISSING_REQUIRED_DEPS=""
  fi

  if [[ ${#missing_optional[@]} -gt 0 ]]; then
    export MISSING_OPTIONAL_DEPS="${missing_optional[*]}"
    echo "[deps] Missing optional dependencies: ${missing_optional[*]}"
  else
    export MISSING_OPTIONAL_DEPS=""
  fi

  if [[ ${#missing_required[@]} -gt 0 ]]; then
    return 1
  else
    return 0
  fi
}

refresh_apt_package_index() {
  local apt_get_bin="${APT_GET_BIN:-apt-get}"
  if ! command -v "${apt_get_bin}" >/dev/null 2>&1; then
    echo "[deps] ERROR: apt-get command not found; cannot refresh package index" >&2
    return 1
  fi

  local refresh_cmd=()
  local refresh_cmd_text
  if [[ "$(id -u)" -eq 0 ]]; then
    refresh_cmd=("${apt_get_bin}" "update")
    refresh_cmd_text="${apt_get_bin} update"
  elif command -v sudo >/dev/null 2>&1; then
    refresh_cmd=("sudo" "${apt_get_bin}" "update")
    refresh_cmd_text="sudo ${apt_get_bin} update"
  else
    echo "[deps] ERROR: sudo command not found and current user is not root" >&2
    echo "[deps] Please refresh apt package index manually as root:" >&2
    echo "       ${apt_get_bin} update" >&2
    return 1
  fi

  echo "[deps] Running: ${refresh_cmd_text}"
  DEBIAN_FRONTEND=noninteractive "${refresh_cmd[@]}"
}

install_system_dependencies() {
  local missing_deps="$1"
  local auto_install="${AUTO_INSTALL_DEPS:-}"

  [[ -z "${missing_deps}" ]] && return 0

  local deps_array=()
  read -ra deps_array <<< "${missing_deps}"
  [[ ${#deps_array[@]} -eq 0 ]] && return 0

  echo "[deps] Installing missing dependencies: ${deps_array[*]}"

  local apt_bin="${APT_GET_BIN:-apt-get}"
  if ! command -v "${apt_bin}" >/dev/null 2>&1; then
    echo "[deps] ERROR: apt-get command not found; please install manually: ${deps_array[*]}" >&2
    return 1
  fi

  local install_cmd=()
  local install_cmd_text
  if [[ "$(id -u)" -eq 0 ]]; then
    install_cmd=("${apt_bin}" "install" "-y")
    install_cmd_text="${apt_bin} install -y"
  elif command -v sudo >/dev/null 2>&1; then
    install_cmd=("sudo" "${apt_bin}" "install" "-y")
    install_cmd_text="sudo ${apt_bin} install -y"
  else
    echo "[deps] ERROR: sudo command not found and current user is not root" >&2
    echo "[deps] Please install manually as root:" >&2
    echo "       ${apt_bin} install -y ${deps_array[*]}" >&2
    return 1
  fi

  if [[ "${auto_install}" != "yes" && "${auto_install}" != "true" ]]; then
    echo "[deps] The following dependencies need to be installed:"
    echo "       ${deps_array[*]}"
    echo -n "[deps] Install now? [Y/n] "
    read -r answer

    if [[ "${answer}" =~ ^[Nn] ]]; then
      echo "[deps] Skipping installation. Please install manually:"
      echo "       ${install_cmd_text} ${deps_array[*]}"
      return 1
    fi
  fi

  if [[ "${install_cmd[0]}" == "sudo" ]] && ! sudo -n true 2>/dev/null; then
    echo "[deps] Requesting sudo privileges for apt install..."
  fi

  if [[ "${SROBOTIS_IN_DOCKER_BUILD:-0}" == "1" || "${APT_UPDATE_BEFORE_INSTALL:-0}" == "1" ]]; then
    if ! refresh_apt_package_index; then
      echo "[deps] ERROR: Failed to refresh apt package index" >&2
      return 1
    fi
  fi

  local legacy_onnxruntime_status=""
  local spacemit_onnxruntime_status=""
  if [[ " ${deps_array[*]} " == *" spacemit-onnxruntime "* ]]; then
    legacy_onnxruntime_status="$(dpkg-query -W -f='${Status}' onnxruntime 2>/dev/null || true)"
    spacemit_onnxruntime_status="$(dpkg-query -W -f='${Status}' spacemit-onnxruntime 2>/dev/null || true)"
  fi
  if [[ " ${deps_array[*]} " == *" spacemit-onnxruntime "* ]] &&
     [[ -n "${legacy_onnxruntime_status}" && "${legacy_onnxruntime_status}" != "install ok not-installed" ]] &&
     [[ "${spacemit_onnxruntime_status}" != "install ok installed" ]]; then
    local remove_cmd=()
    local remove_cmd_text
    if [[ "$(id -u)" -eq 0 ]]; then
      remove_cmd=("${apt_bin}" "remove" "-y" "onnxruntime")
      remove_cmd_text="${apt_bin} remove -y onnxruntime"
    else
      remove_cmd=("sudo" "${apt_bin}" "remove" "-y" "onnxruntime")
      remove_cmd_text="sudo ${apt_bin} remove -y onnxruntime"
    fi
    echo "[deps] Removing conflicting legacy package before installing spacemit-onnxruntime: onnxruntime"
    echo "[deps] Running: ${remove_cmd_text}"
    if ! DEBIAN_FRONTEND=noninteractive "${remove_cmd[@]}"; then
      echo "[deps] ERROR: Failed to remove conflicting package: onnxruntime" >&2
      return 1
    fi
  fi

  echo "[deps] Running: ${install_cmd_text} ${deps_array[*]}"
  if DEBIAN_FRONTEND=noninteractive "${install_cmd[@]}" "${deps_array[@]}"; then
    echo "[deps] Successfully installed dependencies"
    echo "[deps] Verifying installation..."
    for dep in "${deps_array[@]}"; do
      if [[ "$(dpkg-query -W -f='${Status}' "${dep}" 2>/dev/null || true)" == "install ok installed" ]]; then
        echo "[deps] ✓ ${dep}: installed and verified"
      else
        echo "[deps] ⚠ ${dep}: installed but verification failed"
      fi
    done
    return 0
  else
    echo "[deps] ERROR: Failed to install dependencies" >&2
    return 1
  fi
}

check_and_install_dependencies() {
  if [[ "${SROBOTIS_SKIP_DEPS_CHECK:-0}" == "1" ]]; then
    return 0
  fi

  # Always run check (build base deps are collected even without BUILD_CONFIG_FILE).
  # Dependency discovery can improve after installing base tools such as jq, so
  # allow another pass to catch target package dependencies on fresh systems.
  local previous_missing_deps=""
  local attempts_remaining=3
  while [[ ${attempts_remaining} -gt 0 ]]; do
    attempts_remaining=$((attempts_remaining - 1))
    if check_system_dependencies; then
      echo "[deps] All required dependencies are satisfied"
      return 0
    fi

    local missing_deps_str="${MISSING_REQUIRED_DEPS:-}"
    if [[ -z "${missing_deps_str}" ]]; then
      break
    fi

    if [[ "${missing_deps_str}" == "${previous_missing_deps}" ]]; then
      echo "[deps] ERROR: Some dependencies are still missing after installation" >&2
      return 1
    fi
    previous_missing_deps="${missing_deps_str}"

    if ! install_system_dependencies "${missing_deps_str}"; then
      echo "[deps] ERROR: Failed to install required dependencies" >&2
      return 1
    fi

    echo "[deps] Re-checking dependencies after installation..."
  done

  echo "[deps] ERROR: Some dependencies are still missing after installation" >&2
  return 1
}

# Check and install system dependencies for a package build.
# Always checks build base deps first. Optional deps_mode:
# - none: current package only
# - with: current package plus SDK dependency closure
# - only: SDK dependency closure only
check_and_install_dependencies_for_package() {
  local pkg_arg="$1"
  local deps_mode="${2:-none}"
  [[ -z "${pkg_arg}" ]] && return 0

  if [[ "${SROBOTIS_SKIP_DEPS_CHECK:-0}" == "1" ]]; then
    return 0
  fi

  case "${deps_mode}" in
    none|with|only)
      ;;
    *)
      echo "[deps] ERROR: Unknown dependency mode for package check: ${deps_mode}" >&2
      return 1
      ;;
  esac

  local pkg_key
  local pkg_dir_for_type=""
  if [[ -d "${pkg_arg}" ]]; then
    local abs_dir
    abs_dir="$(cd "${pkg_arg}" && pwd)"
    pkg_dir_for_type="${abs_dir}"
    if [[ "${abs_dir}" == "${REPO_ROOT}/"* ]]; then
      pkg_key="${abs_dir#"${REPO_ROOT}"/}"
    else
      pkg_key="${pkg_arg}"
    fi
  else
    pkg_key="${pkg_arg}"
    if [[ -d "${REPO_ROOT}/${pkg_key}" ]]; then
      pkg_dir_for_type="${REPO_ROOT}/${pkg_key}"
    fi
  fi

  echo "[deps] Checking system dependencies for package: ${pkg_key}"

  local deps_lines=()
  # Build base deps (build/package.xml) first so mm also requires build tools and repo.
  if [[ -f "${REPO_ROOT}/build/package.xml" ]] && grep -q '<system_depend' "${REPO_ROOT}/build/package.xml" 2>/dev/null; then
    while IFS='|' read -r dep_type dep_name check_cmd; do
      [[ -z "${dep_type}" || -z "${dep_name}" || -z "${check_cmd}" ]] && continue
      deps_lines+=("${dep_type}|${dep_name}|${check_cmd}")
    done < <(read_package_sysdeps_lines "build")
  fi

  local is_ros2_package=0
  # If this is a ROS2 package (ament_*), also require ROS2 system deps.
  if [[ -n "${pkg_dir_for_type}" && -f "${pkg_dir_for_type}/package.xml" ]]; then
    local build_type
    build_type="$(get_package_build_type "${pkg_dir_for_type}" || true)"
    if [[ "${build_type}" == "ament_cmake" || "${build_type}" == "ament_python" ]]; then
      is_ros2_package=1
      local ros2_deps_xml="${REPO_ROOT}/build/package_ros2.xml"
      if [[ -f "${ros2_deps_xml}" ]] && grep -q '<system_depend' "${ros2_deps_xml}" 2>/dev/null; then
        while IFS='|' read -r dep_type dep_name check_cmd; do
          [[ -z "${dep_type}" || -z "${dep_name}" || -z "${check_cmd}" ]] && continue
          deps_lines+=("${dep_type}|${dep_name}|${check_cmd}")
        done < <(read_sysdeps_lines_from_xml_file "${ros2_deps_xml}")
      fi
    fi
  fi

  if [[ "${deps_mode}" != "only" ]]; then
    while IFS='|' read -r dep_type dep_name check_cmd; do
      [[ -z "${dep_type}" || -z "${dep_name}" || -z "${check_cmd}" ]] && continue
      deps_lines+=("${dep_type}|${dep_name}|${check_cmd}")
    done < <(read_package_sysdeps_lines "${pkg_key}")
  fi

  if [[ "${deps_mode}" != "none" ]]; then
    if ! command -v resolve_nonros2_dependency_closure >/dev/null 2>&1; then
      echo "[deps] ERROR: non-ROS2 dependency resolver is not available" >&2
      return 1
    fi

    local dep_roots=()
    if [[ "${is_ros2_package}" == "1" ]]; then
      mapfile -t dep_roots < <(read_ros2_sdk_nonros2_deps "${pkg_key}")
    else
      dep_roots=("${pkg_key}")
    fi

    local dep_root closure_output dep_pkg
    for dep_root in "${dep_roots[@]}"; do
      [[ -n "${dep_root}" ]] || continue

      local include_dep_root=0
      [[ "${is_ros2_package}" == "1" ]] && include_dep_root=1

      if ! closure_output="$(resolve_nonros2_dependency_closure "${dep_root}" "${include_dep_root}")"; then
        return 1
      fi
      [[ -n "${closure_output}" ]] || continue

      while IFS= read -r dep_pkg; do
        [[ -n "${dep_pkg}" ]] || continue
        while IFS='|' read -r dep_type dep_name check_cmd; do
          [[ -z "${dep_type}" || -z "${dep_name}" || -z "${check_cmd}" ]] && continue
          deps_lines+=("${dep_type}|${dep_name}|${check_cmd}")
        done < <(read_package_sysdeps_lines "${dep_pkg}")
      done <<< "${closure_output}"
    done
  fi

  if [[ ${#deps_lines[@]} -eq 0 ]]; then
    echo "[deps] No system dependencies defined for this package"
    return 0
  fi

  local missing_required=()
  local missing_optional=()
  local checked_packages=()

  for dep_line in "${deps_lines[@]}"; do
    IFS='|' read -r dep_type dep_name check_cmd <<< "${dep_line}"

    local already_checked=false
    for checked in "${checked_packages[@]}"; do
      if [[ "${checked}" == "${dep_name}" ]]; then
        already_checked=true
        break
      fi
    done
    [[ "${already_checked}" == "true" ]] && continue

    if check_single_dependency "${dep_name}" "${check_cmd}"; then
      echo "[deps] ✓ ${dep_name}: found"
      checked_packages+=("${dep_name}")
    else
      echo "[deps] ✗ ${dep_name}: missing (${dep_type})"
      checked_packages+=("${dep_name}")
      if [[ "${dep_type}" == "required" ]]; then
        missing_required+=("${dep_name}")
      else
        missing_optional+=("${dep_name}")
      fi
    fi
  done

  if [[ ${#missing_optional[@]} -gt 0 ]]; then
    echo "[deps] Missing optional dependencies: ${missing_optional[*]}"
  fi

  if [[ ${#missing_required[@]} -eq 0 ]]; then
    echo "[deps] All required dependencies are satisfied"
    return 0
  fi

  echo "[deps] Missing required dependencies: ${missing_required[*]}"
  local missing_required_str="${missing_required[*]}"
  if install_system_dependencies "${missing_required_str}"; then
    echo "[deps] Re-checking dependencies after installation..."
    for dep_line in "${deps_lines[@]}"; do
      IFS='|' read -r dep_type dep_name check_cmd <<< "${dep_line}"
      if [[ "${dep_type}" == "required" ]] && ! check_single_dependency "${dep_name}" "${check_cmd}"; then
        echo "[deps] ERROR: ${dep_name} still missing after installation" >&2
        return 1
      fi
    done
    echo "[deps] All dependencies are now satisfied"
    return 0
  else
    echo "[deps] ERROR: Failed to install required dependencies" >&2
    return 1
  fi
}
