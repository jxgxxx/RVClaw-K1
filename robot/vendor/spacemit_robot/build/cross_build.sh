#!/usr/bin/env bash
#
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_ROOT="${OUTPUT_ROOT:-${REPO_ROOT}/output}"

export SROBOTIS_CROSS_BUILD=1

CM_BUILD_ROOT="${CM_BUILD_ROOT:-${OUTPUT_ROOT}/build/cmake}"
ROS2_BUILD_ROOT="${ROS2_BUILD_ROOT:-${OUTPUT_ROOT}/build/ros2}"
ROS2_LOG_ROOT="${ROS2_LOG_ROOT:-${OUTPUT_ROOT}/log/ros2}"
STAGING_PREFIX="${STAGING_PREFIX:-${OUTPUT_ROOT}/staging}"
ROOTFS_PREFIX="${ROOTFS_PREFIX:-${OUTPUT_ROOT}/rootfs}"
PREFIX="${PREFIX:-${STAGING_PREFIX}}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
PARALLEL_JOBS="${PARALLEL_JOBS:-$(nproc)}"
BUILD_TARGET="${BUILD_TARGET:-}"
BUILD_TARGET_FILE="${BUILD_TARGET_FILE:-}"

source "${REPO_ROOT}/build/common.sh"
source "${REPO_ROOT}/build/nonros2.sh"
source "${REPO_ROOT}/build/ros2.sh"

CROSS_DEP_SEP="|"
CROSS_HOST_ARCH="${SROBOTIS_CROSS_HOST_ARCH:-$(normalize_build_arch "$(uname -m)")}"
CROSS_TARGET_ARCH="${SROBOTIS_CROSS_TARGET_ARCH:-riscv64}"
CROSS_RUST_VERSION="${SROBOTIS_CROSS_RUST_VERSION:-1.91}"
CROSS_CARGO_EXECUTABLE="${SROBOTIS_CROSS_CARGO_EXECUTABLE:-/usr/bin/cargo-${CROSS_RUST_VERSION}}"
CROSS_RUSTC_EXECUTABLE="${SROBOTIS_CROSS_RUSTC_EXECUTABLE:-/usr/bin/rustc-${CROSS_RUST_VERSION}}"

CROSS_OUTPUT_ROOT=""
CROSS_SYSROOT=""
CROSS_HOST_PREFIX=""
CROSS_STAGING_PREFIX=""
CROSS_ROOTFS_PREFIX=""
CROSS_TOOLCHAIN_FILE=""
CROSS_MESON_CROSS_FILE=""
CROSS_PYTHON_CMAKE_INIT=""
CROSS_BIANBU_TAG=""
CROSS_BIANBU_IMAGE=""
CROSS_UBUNTU_IMAGE=""

HOST_DEP_LINES=()
SYSROOT_DEP_LINES=()

cross_usage() {
  cat <<EOF
Usage: cross_build.sh <command> [args...]

Commands:
  deps all                      Print cross dependency host/target split for selected target
  deps package <dir>            Print cross dependency host/target split for a package
  all                           Cross build all packages from selected k1/k3 target
  cmake, C                      Cross build non-ROS2 packages from selected target
  ros2, R                       Cross build ROS2 packages from selected target
  package <dir> [clean|--deps|--with-deps]
                                Cross build or clean one package
  clean [all|cmake|ros2]        Clean cross output
  deploy-rootfs, rootfs         Generate runtime rootfs from cross staging
  runtime-deps [all]            Print board-side apt runtime dependencies
  runtime-deps package <dir>    Validate package path, then scan current cross output

Dependency ownership:
  - package.xml owns package-level system dependencies
  - build/package.xml owns build-system base dependencies
  - build/package_cross.xml owns cross-only global build dependencies
  - build/package_ros2_cross.xml owns cross-only ROS2 global dependencies
EOF
}

cross_error() {
  echo "[cross] ERROR: $*" >&2
}

cross_pkg_key_from_arg() {
  local pkg_arg="$1"
  local abs_dir
  if [[ -d "${pkg_arg}" ]]; then
    abs_dir="$(cd "${pkg_arg}" && pwd)"
    if [[ "${abs_dir}" == "${REPO_ROOT}/"* ]]; then
      printf '%s\n' "${abs_dir#"${REPO_ROOT}"/}"
    else
      printf '%s\n' "${pkg_arg}"
    fi
  else
    printf '%s\n' "${pkg_arg%/}"
  fi
}

cross_package_exists() {
  local pkg_key="$1"
  local xml
  [[ -d "${REPO_ROOT}/${pkg_key}" ]] && return 0
  xml="$(package_xml_path "${pkg_key}")"
  [[ -f "${xml}" ]] && return 0
  return 1
}

cross_validate_package_path() {
  local pkg_key="$1"
  if ! cross_package_exists "${pkg_key}"; then
    cross_error "Package path not found: ${pkg_key}"
    return 1
  fi
}

cross_is_ros2_pkg_key() {
  local pkg_key="$1"
  [[ "${pkg_key}" == middleware/ros2/* || "${pkg_key}" == application/ros2/* ]]
}

cross_is_ros2_package_dir() {
  local pkg_key="$1"
  local pkg_dir="${REPO_ROOT}/${pkg_key}"
  local build_type=""
  [[ -f "${pkg_dir}/package.xml" ]] || return 1
  build_type="$(get_package_build_type "${pkg_dir}" || true)"
  [[ "${build_type}" == "ament_cmake" || "${build_type}" == "ament_python" ]]
}

cross_default_realm_for_pkg() {
  local pkg_key="$1"
  case "${pkg_key}" in
    build|build_cross|build_ros2_cross)
      echo "host"
      ;;
    *)
      echo "target"
      ;;
  esac
}

cross_realm_arch() {
  local realm="$1"
  case "${realm}" in
    host)
      normalize_build_arch "${CROSS_HOST_ARCH}"
      ;;
    target)
      normalize_build_arch "${CROSS_TARGET_ARCH}"
      ;;
    *)
      echo ""
      ;;
  esac
}

cross_arch_matches_realm() {
  local realm="$1"
  local arch_filter="${2:-}"
  [[ -z "${arch_filter}" ]] && return 0

  local wanted item
  wanted="$(cross_realm_arch "${realm}")"
  arch_filter="${arch_filter//,/ }"
  for item in ${arch_filter}; do
    [[ "$(normalize_build_arch "${item}")" == "${wanted}" ]] && return 0
  done
  return 1
}

cross_when_matches() {
  local when="${1:-all}"
  case "${when:-all}" in
    all|cross)
      return 0
      ;;
    native|docker)
      return 1
      ;;
    *)
      cross_error "Invalid when='${when}'"
      return 2
      ;;
  esac
}

cross_required_type_from_line() {
  deps_required_type_from_line "$@"
}

cross_validate_dep_metadata() {
  local pkg_key="$1"
  local dep_name="$2"
  local arch_filter="$3"
  local realm="$4"
  local when="$5"
  local check_kind="$6"

  if [[ "${arch_filter}" == "cross" ]]; then
    cross_error "arch='cross' is not allowed for ${dep_name} in ${pkg_key}; use when='cross' instead"
    return 1
  fi

  case "${realm:-}" in
    ""|host|target|both|skip)
      ;;
    *)
      cross_error "Invalid realm='${realm}' for ${dep_name} in ${pkg_key}"
      return 1
      ;;
  esac

  case "${when:-all}" in
    all|native|docker|cross)
      ;;
    *)
      cross_error "Invalid when='${when}' for ${dep_name} in ${pkg_key}"
      return 1
      ;;
  esac

  case "${check_kind:-}" in
    ""|dpkg|command|pkg-config|file|rustlib)
      ;;
    *)
      cross_error "Invalid check_kind='${check_kind}' for ${dep_name} in ${pkg_key}"
      return 1
      ;;
  esac
}

cross_normalize_check() {
  local dep_name="$1"
  local check_kind="${2:-}"
  local check_arg="${3:-}"
  local legacy_check="${4:-}"

  if [[ -n "${legacy_check}" && -z "${check_kind}" ]]; then
    printf 'legacy%s%s\n' "${CROSS_DEP_SEP}" "${legacy_check}"
    return 0
  fi

  check_kind="${check_kind:-dpkg}"
  case "${check_kind}" in
    dpkg)
      check_arg="${check_arg:-${dep_name}}"
      ;;
    command|pkg-config|file)
      check_arg="${check_arg:-${dep_name}}"
      ;;
    rustlib)
      check_arg="${check_arg:-riscv64*-unknown-linux-gnu}"
      ;;
  esac
  printf '%s%s%s\n' "${check_kind}" "${CROSS_DEP_SEP}" "${check_arg}"
}

cross_check_display_cmd() {
  local dep_name="$1"
  local check_kind="$2"
  local check_arg="$3"
  local quoted
  case "${check_kind}" in
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
      printf 'find /usr/lib -path "*/rustlib/%s/lib/libcore-*.rlib"\n' "${check_arg}"
      ;;
    legacy)
      printf '%s\n' "${check_arg}"
      ;;
    *)
      cross_error "Invalid check_kind='${check_kind}' for ${dep_name}"
      return 1
      ;;
  esac
}

read_cross_xml_sysdeps() {
  local pkg_key="$1"
  local pkg_xml="$2"
  [[ -f "${pkg_xml}" ]] || return 0

  while IFS= read -r line; do
    [[ -z "${line}" ]] && continue
    local name dep_type legacy_check arch_filter board_filter realm when check_kind check_arg option_key option_value
    name="$(echo "${line}" | sed -n 's/.*> *\([^<]*\) *<\/system_depend>.*/\1/p' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
    [[ -z "${name}" ]] && continue

    legacy_check="$(xml_line_attr "${line}" "check")"
    arch_filter="$(xml_line_attr "${line}" "arch")"
    board_filter="$(xml_line_attr "${line}" "board")"
    realm="$(xml_line_attr "${line}" "realm")"
    when="$(xml_line_attr "${line}" "when")"
    check_kind="$(xml_line_attr "${line}" "check_kind")"
    check_arg="$(xml_line_attr "${line}" "check_arg")"
    option_key="$(xml_line_attr "${line}" "option_key")"
    option_value="$(xml_line_attr "${line}" "option_value")"

    cross_validate_dep_metadata "${pkg_key}" "${name}" "${arch_filter}" "${realm}" "${when}" "${check_kind}" || return 1
    dep_type="$(cross_required_type_from_line "${line}")" || return 1

    printf '%s\n' "${pkg_key}${CROSS_DEP_SEP}${dep_type}${CROSS_DEP_SEP}${name}${CROSS_DEP_SEP}${check_kind}${CROSS_DEP_SEP}${check_arg}${CROSS_DEP_SEP}${arch_filter}${CROSS_DEP_SEP}${board_filter}${CROSS_DEP_SEP}${realm}${CROSS_DEP_SEP}${when}${CROSS_DEP_SEP}${option_key}${CROSS_DEP_SEP}${option_value}${CROSS_DEP_SEP}${legacy_check}"
  done < <(grep -E '<system_depend' "${pkg_xml}" 2>/dev/null)
}

read_cross_package_sysdeps() {
  local pkg_key="$1"
  local pkg_xml
  pkg_xml="$(package_xml_path "${pkg_key}")"
  read_cross_xml_sysdeps "${pkg_key}" "${pkg_xml}"
}

cross_add_global_deps() {
  if [[ -f "${REPO_ROOT}/build/package.xml" ]] && grep -q '<system_depend' "${REPO_ROOT}/build/package.xml" 2>/dev/null; then
    read_cross_xml_sysdeps "build" "${REPO_ROOT}/build/package.xml"
  fi
  if [[ -f "${REPO_ROOT}/build/package_cross.xml" ]] && grep -q '<system_depend' "${REPO_ROOT}/build/package_cross.xml" 2>/dev/null; then
    read_cross_xml_sysdeps "build_cross" "${REPO_ROOT}/build/package_cross.xml"
  fi
}

cross_add_ros2_global_deps_if_needed() {
  local include_ros2="$1"
  if [[ "${include_ros2}" == "1" && -f "${REPO_ROOT}/build/package_ros2_cross.xml" ]] && \
    grep -q '<system_depend' "${REPO_ROOT}/build/package_ros2_cross.xml" 2>/dev/null; then
    read_cross_xml_sysdeps "build_ros2_cross" "${REPO_ROOT}/build/package_ros2_cross.xml"
  fi
}

cross_collect_package_keys_for_package() {
  local pkg_key="$1"
  local include_root="${2:-1}"
  local include_deps="${3:-1}"
  cross_validate_package_path "${pkg_key}" || return 1

  if [[ "${include_root}" == "1" ]]; then
    printf '%s\n' "${pkg_key}"
  fi
  [[ "${include_deps}" == "1" ]] || return 0

  if cross_is_ros2_package_dir "${pkg_key}"; then
    local sdk_dep closure dep_pkg
    while IFS= read -r sdk_dep; do
      [[ -n "${sdk_dep}" ]] || continue
      if closure="$(resolve_nonros2_dependency_closure "${sdk_dep}" 1)"; then
        while IFS= read -r dep_pkg; do
          [[ -n "${dep_pkg}" ]] && printf '%s\n' "${dep_pkg}"
        done <<< "${closure}"
      else
        return 1
      fi
    done < <(read_ros2_sdk_nonros2_deps "${pkg_key}")
  elif ! cross_is_ros2_pkg_key "${pkg_key}"; then
    local closure dep_pkg
    if closure="$(resolve_nonros2_dependency_closure "${pkg_key}" 0)"; then
      while IFS= read -r dep_pkg; do
        [[ -n "${dep_pkg}" ]] && printf '%s\n' "${dep_pkg}"
      done <<< "${closure}"
    else
      return 1
    fi
  fi
}

cross_target_needs_ros2_for_cmd() {
  local cmd="$1"
  case "${cmd}" in
    ros2|R)
      return 0
      ;;
    all)
      target_needs_ros2
      return $?
      ;;
    *)
      return 1
      ;;
  esac
}

cross_collect_scope_deps() {
  local cmd="$1"
  shift || true

  local include_ros2=0
  local pkg_key pkg

  cross_add_global_deps

  case "${cmd}" in
    deps)
      local deps_scope="${1:-all}"
      case "${deps_scope}" in
        all)
          load_build_config >/dev/null
          if [[ -z "${BUILD_CONFIG_FILE:-}" || ! -f "${BUILD_CONFIG_FILE}" ]]; then
            cross_error "deps all requires BUILD_TARGET or BUILD_TARGET_FILE"
            return 1
          fi
          if target_needs_ros2; then
            include_ros2=1
          fi
          cross_add_ros2_global_deps_if_needed "${include_ros2}"
          has_jq || { cross_error "jq is required for target JSON parsing"; return 1; }
          while IFS= read -r pkg; do
            [[ -n "${pkg}" ]] || continue
            cross_validate_package_path "${pkg}" || return 1
            read_cross_package_sysdeps "${pkg}"
          done < <(resolve_enabled_with_metadata)
          ;;
        package)
          pkg_key="$(cross_pkg_key_from_arg "${2:-}")"
          [[ -n "${pkg_key}" ]] || { cross_error "deps package requires a package path"; return 1; }
          if [[ -n "${BUILD_TARGET:-}${BUILD_TARGET_FILE:-}" ]]; then
            load_build_config >/dev/null
          fi
          if cross_is_ros2_package_dir "${pkg_key}"; then
            include_ros2=1
          fi
          cross_add_ros2_global_deps_if_needed "${include_ros2}"
          while IFS= read -r pkg; do
            [[ -n "${pkg}" ]] || continue
            read_cross_package_sysdeps "${pkg}"
          done < <(cross_collect_package_keys_for_package "${pkg_key}")
          ;;
        *)
          cross_error "Unknown deps scope: ${deps_scope}"
          return 1
          ;;
      esac
      ;;
    all|cmake|C|ros2|R)
      load_build_config >/dev/null
      if [[ -z "${BUILD_CONFIG_FILE:-}" || ! -f "${BUILD_CONFIG_FILE}" ]]; then
        cross_error "${cmd} requires BUILD_TARGET or BUILD_TARGET_FILE"
        return 1
      fi
      if cross_target_needs_ros2_for_cmd "${cmd}"; then
        include_ros2=1
      fi
      cross_add_ros2_global_deps_if_needed "${include_ros2}"
      has_jq || { cross_error "jq is required for target JSON parsing"; return 1; }
      while IFS= read -r pkg; do
        [[ -n "${pkg}" ]] || continue
        case "${cmd}" in
          cmake|C)
            cross_is_ros2_pkg_key "${pkg}" && continue
            ;;
          ros2|R)
            cross_is_ros2_pkg_key "${pkg}" || continue
            ;;
        esac
        cross_validate_package_path "${pkg}" || return 1
        read_cross_package_sysdeps "${pkg}"
      done < <(resolve_enabled_with_metadata)
      ;;
    package|pkg)
      pkg_key="$(cross_pkg_key_from_arg "${1:-}")"
      [[ -n "${pkg_key}" ]] || { cross_error "package requires a package path"; return 1; }
      local action="${2:-build}"
      local extra_arg="${3:-}"
      local include_root=1
      local include_deps=0
      if [[ -n "${extra_arg}" ]]; then
        cross_error "Unexpected package argument: ${extra_arg}"
        return 1
      fi
      case "${action}" in
        --deps)
          include_root=0
          include_deps=1
          ;;
        --with-deps)
          include_deps=1
          ;;
        clean)
          return 0
          ;;
        build|"")
          ;;
        *)
          cross_error "Unknown package action: ${action}"
          return 1
          ;;
      esac
      if [[ -n "${BUILD_TARGET:-}${BUILD_TARGET_FILE:-}" ]]; then
        load_build_config >/dev/null
      fi
      if cross_is_ros2_package_dir "${pkg_key}"; then
        include_ros2=1
      fi
      cross_add_ros2_global_deps_if_needed "${include_ros2}"
      while IFS= read -r pkg; do
        [[ -n "${pkg}" ]] || continue
        read_cross_package_sysdeps "${pkg}"
      done < <(cross_collect_package_keys_for_package "${pkg_key}" "${include_root}" "${include_deps}")
      ;;
    *)
      ;;
  esac
}

cross_append_split_dep() {
  local realm="$1"
  local dep_type="$2"
  local dep_name="$3"
  local check_kind="$4"
  local check_arg="$5"
  local line="${dep_type}${CROSS_DEP_SEP}${dep_name}${CROSS_DEP_SEP}${check_kind}${CROSS_DEP_SEP}${check_arg}"
  case "${realm}" in
    host)
      HOST_DEP_LINES+=("${line}")
      ;;
    target)
      SYSROOT_DEP_LINES+=("${line}")
      ;;
  esac
}

cross_dep_option_matches() {
  local pkg_key="$1"
  local option_key="${2:-}"
  local option_value="${3:-}"
  [[ -z "${option_key}" ]] && return 0

  local config_file="${BUILD_CONFIG_FILE:-}"
  if [[ -z "${config_file}" || ! -f "${config_file}" ]] || ! has_jq; then
    return 0
  fi

  local selected=()
  local wanted item
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

split_cross_dependencies() {
  local cmd="$1"
  shift || true

  HOST_DEP_LINES=()
  SYSROOT_DEP_LINES=()
  if [[ -z "${BUILD_CONFIG_FILE:-}" && -n "${BUILD_TARGET:-}${BUILD_TARGET_FILE:-}" ]]; then
    load_build_config >/dev/null
  fi
  local raw_lines=()
  local raw_output=""
  if ! raw_output="$(cross_collect_scope_deps "${cmd}" "$@")"; then
    return 1
  fi
  if [[ -n "${raw_output}" ]]; then
    mapfile -t raw_lines <<< "${raw_output}"
  fi

  declare -A seen=()
  local raw pkg_key dep_type dep_name check_kind check_arg arch_filter board_filter realm when option_key option_value legacy_check check_info resolved_realm key
  for raw in "${raw_lines[@]}"; do
    [[ -n "${raw}" ]] || continue
    IFS="${CROSS_DEP_SEP}" read -r pkg_key dep_type dep_name check_kind check_arg arch_filter board_filter realm when option_key option_value legacy_check <<< "${raw}"
    [[ -n "${dep_name:-}" ]] || continue

    cross_validate_dep_metadata "${pkg_key}" "${dep_name}" "${arch_filter}" "${realm}" "${when}" "${check_kind}" || return 1
    cross_when_matches "${when:-all}" || continue
    xml_board_matches_current "${board_filter}" || continue
    cross_dep_option_matches "${pkg_key}" "${option_key}" "${option_value}" || continue
    [[ "${realm}" == "skip" ]] && continue

    check_info="$(cross_normalize_check "${dep_name}" "${check_kind}" "${check_arg}" "${legacy_check}")" || return 1
    IFS="${CROSS_DEP_SEP}" read -r check_kind check_arg <<< "${check_info}"

    resolved_realm="${realm:-$(cross_default_realm_for_pkg "${pkg_key}")}"
    case "${resolved_realm}" in
      host|target)
        if cross_arch_matches_realm "${resolved_realm}" "${arch_filter}"; then
          key="${resolved_realm}${CROSS_DEP_SEP}${dep_name}${CROSS_DEP_SEP}${check_kind}${CROSS_DEP_SEP}${check_arg}"
          if [[ -z "${seen[${key}]:-}" ]]; then
            seen["${key}"]=1
            cross_append_split_dep "${resolved_realm}" "${dep_type}" "${dep_name}" "${check_kind}" "${check_arg}"
          fi
        fi
        ;;
      both)
        if cross_arch_matches_realm "host" "${arch_filter}"; then
          key="host${CROSS_DEP_SEP}${dep_name}${CROSS_DEP_SEP}${check_kind}${CROSS_DEP_SEP}${check_arg}"
          if [[ -z "${seen[${key}]:-}" ]]; then
            seen["${key}"]=1
            cross_append_split_dep "host" "${dep_type}" "${dep_name}" "${check_kind}" "${check_arg}"
          fi
        fi
        if cross_arch_matches_realm "target" "${arch_filter}"; then
          key="target${CROSS_DEP_SEP}${dep_name}${CROSS_DEP_SEP}${check_kind}${CROSS_DEP_SEP}${check_arg}"
          if [[ -z "${seen[${key}]:-}" ]]; then
            seen["${key}"]=1
            cross_append_split_dep "target" "${dep_type}" "${dep_name}" "${check_kind}" "${check_arg}"
          fi
        fi
        ;;
      *)
        cross_error "Invalid dependency realm for ${dep_name}: ${resolved_realm}"
        return 1
        ;;
    esac
  done
}

print_cross_dep_section() {
  local title="$1"
  shift || true
  local deps=("$@")
  local dep_line dep_type dep_name check_kind check_arg check_cmd
  echo "${title}"
  if [[ ${#deps[@]} -eq 0 ]]; then
    echo "  (none)"
    return 0
  fi
  for dep_line in "${deps[@]}"; do
    IFS="${CROSS_DEP_SEP}" read -r dep_type dep_name check_kind check_arg <<< "${dep_line}"
    check_cmd="$(cross_check_display_cmd "${dep_name}" "${check_kind}" "${check_arg}")" || return 1
    printf '  - %s [%s] check_kind=%s check_arg=%s check=%s\n' \
      "${dep_name}" "${dep_type}" "${check_kind}" "${check_arg}" "${check_cmd}"
  done
}

print_cross_deps_split() {
  print_cross_dep_section "[cross] Host dependencies:" "${HOST_DEP_LINES[@]}"
  print_cross_dep_section "[cross] Target sysroot dependencies:" "${SYSROOT_DEP_LINES[@]}"
}

select_cross_images() {
  local board=""
  local ubuntu_tag=""
  if [[ -n "${BUILD_CONFIG_FILE:-}" && -f "${BUILD_CONFIG_FILE}" ]] && has_jq; then
    board="$(jq -r '.board // ""' "${BUILD_CONFIG_FILE}" 2>/dev/null || true)"
  fi

  case "${BUILD_TARGET} ${board}" in
    *k1*|*K1*)
      CROSS_BIANBU_TAG="${SROBOTIS_CROSS_BIANBU_TAG:-2.3}"
      ubuntu_tag="24.04"
      ;;
    *k3*|*K3*)
      CROSS_BIANBU_TAG="${SROBOTIS_CROSS_BIANBU_TAG:-4.0}"
      ubuntu_tag="26.04"
      ;;
    *)
      cross_error "Cannot select Bianbu image without a k1/k3 target"
      return 1
      ;;
  esac

  CROSS_UBUNTU_IMAGE="${SROBOTIS_CROSS_UBUNTU_IMAGE:-ubuntu:${ubuntu_tag}}"
  CROSS_BIANBU_IMAGE="${SROBOTIS_CROSS_BIANBU_IMAGE:-bianbu:${CROSS_BIANBU_TAG}}"
}

init_cross_paths() {
  local target_name="${BUILD_TARGET:-adhoc}"
  CROSS_OUTPUT_ROOT="${SROBOTIS_CROSS_OUTPUT_ROOT:-${OUTPUT_ROOT}/cross/${target_name}}"
  CROSS_SYSROOT="${SROBOTIS_CROSS_SYSROOT:-${CROSS_OUTPUT_ROOT}/sysroot}"
  CROSS_HOST_PREFIX="${SROBOTIS_CROSS_HOST_PREFIX:-${CROSS_OUTPUT_ROOT}/host}"
  CROSS_STAGING_PREFIX="${CROSS_OUTPUT_ROOT}/staging"
  CROSS_ROOTFS_PREFIX="${CROSS_OUTPUT_ROOT}/rootfs"
  CROSS_TOOLCHAIN_FILE="${CROSS_OUTPUT_ROOT}/toolchain-riscv64.cmake"
  CROSS_MESON_CROSS_FILE="${CROSS_OUTPUT_ROOT}/meson-riscv64.ini"
  CROSS_PYTHON_CMAKE_INIT="${CROSS_OUTPUT_ROOT}/cross-python-init.cmake"
}

cross_container_name() {
  local role="$1"
  local image=""
  local hash
  case "${role}" in
    ubuntu)
      image="${CROSS_UBUNTU_IMAGE:-}"
      ;;
    bianbu)
      image="${CROSS_BIANBU_IMAGE:-}"
      ;;
  esac
  hash="$(printf '%s' "${REPO_ROOT}:${BUILD_TARGET:-adhoc}:${role}:${image}" | sha1sum | awk '{print substr($1,1,12)}')"
  printf 'srobotis-cross-%s-%s\n' "${role}" "${hash}"
}

cross_container_has_repo_root() {
  local name="$1"

  docker exec -i \
    --workdir / \
    "${name}" \
    test -f "${REPO_ROOT}/build/build.sh" >/dev/null 2>&1
}

ensure_docker_available() {
  command -v docker >/dev/null 2>&1 || { cross_error "docker command not found"; return 1; }
  docker info >/dev/null 2>&1 || { cross_error "Docker daemon is not available"; return 1; }
}

ensure_docker_image() {
  local image="$1"
  if docker image inspect "${image}" >/dev/null 2>&1; then
    return 0
  fi
  echo "[cross] Pulling image: ${image}"
  docker pull "${image}"
}

ensure_cross_bianbu_image() {
  if [[ -n "${SROBOTIS_CROSS_BIANBU_IMAGE:-}" ]]; then
    ensure_docker_image "${CROSS_BIANBU_IMAGE}"
    return $?
  fi

  local short_image="bianbu:${CROSS_BIANBU_TAG}"
  local remote_image="harbor.spacemit.com/bianbu/bianbu:${CROSS_BIANBU_TAG}"
  CROSS_BIANBU_IMAGE="${short_image}"

  if docker image inspect "${short_image}" >/dev/null 2>&1; then
    return 0
  fi

  if docker image inspect "${remote_image}" >/dev/null 2>&1; then
    docker tag "${remote_image}" "${short_image}" >/dev/null 2>&1 || \
      CROSS_BIANBU_IMAGE="${remote_image}"
    return 0
  fi

  echo "[cross] Image ${short_image} not found. Pulling ${remote_image} ..."
  if ! docker pull "${remote_image}"; then
    cross_error "Failed to pull ${remote_image}"
    return 1
  fi

  docker tag "${remote_image}" "${short_image}" >/dev/null 2>&1 || \
    CROSS_BIANBU_IMAGE="${remote_image}"
}

ensure_container() {
  local name="$1"
  local image="$2"
  local platform="${3:-}"
  local run_args=(-d --name "${name}" -v "${REPO_ROOT}:${REPO_ROOT}" -w "${REPO_ROOT}")
  mkdir -p "${CROSS_OUTPUT_ROOT}" "${CROSS_HOST_PREFIX}"
  if [[ -n "${platform}" ]]; then
    run_args+=(--platform "${platform}")
  fi
  if docker container inspect "${name}" >/dev/null 2>&1; then
    if docker start "${name}" >/dev/null; then
      if cross_container_has_repo_root "${name}"; then
        return 0
      fi
      echo "[cross] Recreating container ${name}: ${REPO_ROOT} is not mounted"
    else
      echo "[cross] Recreating container ${name}: failed to start existing container"
    fi
    docker rm -f "${name}" >/dev/null || return $?
  fi
  docker run "${run_args[@]}" "${image}" sleep infinity >/dev/null
}

configure_bianbu_container() {
  local container="$1"

  case "${CROSS_BIANBU_TAG}" in
    2.3)
      docker exec "${container}" bash -lc '
        set -euo pipefail
        src="/etc/apt/sources.list.d/bianbu.sources"
        [[ -f "${src}" ]] || exit 0
        if grep -Eq "^[[:space:]]*Suites:.*(^|[[:space:]])bianbu-v2\.2-updates([[:space:]]|$)" "${src}"; then
          echo "[cross] Replacing bianbu-v2.2-updates with bianbu-v2.3-updates in ${src}"
          sed -i "/^[[:space:]]*Suites:/ s/bianbu-v2\.2-updates/bianbu-v2.3-updates/g" "${src}"
        elif ! grep -Eq "^[[:space:]]*Suites:.*(^|[[:space:]])bianbu-v2\.3-updates([[:space:]]|$)" "${src}"; then
          echo "[cross] Adding bianbu-v2.3-updates apt suite to ${src}"
          sed -i "/^[[:space:]]*Suites:/ s/$/ bianbu-v2.3-updates/" "${src}"
        fi
        if ! grep -Eq "^[[:space:]]*Suites:.*(^|[[:space:]])noble-ros([[:space:]]|$)" "${src}"; then
          echo "[cross] Adding noble-ros apt suite to ${src}"
          sed -i "/^[[:space:]]*Suites:/ s/$/ noble-ros/" "${src}"
        fi
        pref="/etc/apt/preferences.d/bianbu"
        mkdir -p "$(dirname "${pref}")"
        touch "${pref}"
        if grep -Eq "^[[:space:]]*Pin:.*n=bianbu-v2\.2-updates([[:space:]]|$)" "${pref}"; then
          echo "[cross] Replacing bianbu-v2.2-updates apt pin with bianbu-v2.3-updates in ${pref}"
          sed -i "s/n=bianbu-v2\.2-updates/n=bianbu-v2.3-updates/g" "${pref}"
        fi
        if ! grep -Eq "^[[:space:]]*Pin:.*n=bianbu-v2\.3-updates([[:space:]]|$)" "${pref}"; then
          echo "[cross] Adding bianbu-v2.3-updates apt pin to ${pref}"
          cat >> "${pref}" <<EOF

Package: *
Pin: release o=Spacemit, n=bianbu-v2.3-updates
Pin-Priority: 1100
EOF
        fi
      '
      ;;
  esac
}

container_check_dependency() {
  local container="$1"
  local dep_name="$2"
  local check_kind="$3"
  local check_arg="$4"
  case "${check_kind}" in
    dpkg)
      docker exec "${container}" bash -lc 'test "$(dpkg-query -W -f=\${Status} "$1" 2>/dev/null || true)" = "install ok installed"' _ "${check_arg:-${dep_name}}" >/dev/null 2>&1
      ;;
    command)
      docker exec "${container}" bash -lc "command -v \"\$1\" >/dev/null" _ "${check_arg:-${dep_name}}" >/dev/null 2>&1
      ;;
    pkg-config)
      docker exec "${container}" bash -lc "pkg-config --exists \"\$1\"" _ "${check_arg:-${dep_name}}" >/dev/null 2>&1
      ;;
    file)
      docker exec "${container}" bash -lc "test -e \"\$1\"" _ "${check_arg}" >/dev/null 2>&1
      ;;
    rustlib)
      docker exec "${container}" bash -lc \
        'compgen -G "/usr/lib/*/rustlib/$1/lib/libcore-*.rlib" >/dev/null || compgen -G "/usr/lib/rustlib/$1/lib/libcore-*.rlib" >/dev/null || find /usr/lib -path "*/rustlib/$1/lib/libcore-*.rlib" -print -quit | grep -q .' \
        _ "${check_arg:-riscv64*-unknown-linux-gnu}" >/dev/null 2>&1
      ;;
    legacy)
      docker exec "${container}" bash -lc "${check_arg}" >/dev/null 2>&1
      ;;
    *)
      return 1
      ;;
  esac
}

install_deps_in_container() {
  local container="$1"
  local label="$2"
  shift 2 || true
  local dep_lines=("$@")
  local missing=()
  local dep_line dep_type dep_name check_kind check_arg
  local install_key
  declare -A install_seen=()

  echo "[cross] Checking ${label} dependencies..."
  for dep_line in "${dep_lines[@]}"; do
    IFS="${CROSS_DEP_SEP}" read -r dep_type dep_name check_kind check_arg <<< "${dep_line}"
    if container_check_dependency "${container}" "${dep_name}" "${check_kind}" "${check_arg}"; then
      echo "[cross] ${label}: ${dep_name} found"
      continue
    fi
    echo "[cross] ${label}: ${dep_name} missing (${dep_type})"
    if [[ "${dep_type}" == "required" ]]; then
      install_key="${dep_name}"
      if [[ -z "${install_seen[${install_key}]:-}" ]]; then
        install_seen["${install_key}"]=1
        missing+=("${dep_name}")
      fi
    fi
  done

  if [[ ${#missing[@]} -eq 0 ]]; then
    return 0
  fi

  echo "[cross] Installing ${label} dependencies: ${missing[*]}"
  if [[ " ${missing[*]} " == *" spacemit-onnxruntime "* ]]; then
    docker exec "${container}" bash -lc '
      set -euo pipefail
      export DEBIAN_FRONTEND=noninteractive
      legacy_status="$(dpkg-query -W -f=\${Status} onnxruntime 2>/dev/null || true)"
      spacemit_status="$(dpkg-query -W -f=\${Status} spacemit-onnxruntime 2>/dev/null || true)"
      if [[ -n "${legacy_status}" && "${legacy_status}" != "install ok not-installed" ]] &&
         [[ "${spacemit_status}" != "install ok installed" ]]; then
        apt-get remove -y onnxruntime
      fi
    '
  fi
  docker exec "${container}" bash -lc 'export DEBIAN_FRONTEND=noninteractive; apt-get update && apt-get install -y "$@"' _ "${missing[@]}"
}

cross_sysroot_deps_fingerprint() {
  printf '%s\n' "${SYSROOT_DEP_LINES[@]}" | sort | sha1sum | awk '{print $1}'
}

cross_sysroot_deps_stamp_file() {
  printf '%s\n' "${CROSS_OUTPUT_ROOT}/.sysroot-deps.sha1"
}

cross_sysroot_ready() {
  [[ -d "${CROSS_SYSROOT}/usr" ]]
}

sync_bianbu_sysroot() {
  local container="$1"
  local force="${2:-0}"
  local fingerprint="$3"
  local stamp
  stamp="$(cross_sysroot_deps_stamp_file)"

  if [[ "${SROBOTIS_CROSS_SKIP_SYSROOT_SYNC:-0}" == "1" ]] && cross_sysroot_ready; then
    echo "[cross] Reusing existing sysroot: ${CROSS_SYSROOT}"
    return 0
  fi
  if [[ "${force}" != "1" && -f "${stamp}" && "$(cat "${stamp}")" == "${fingerprint}" ]] && cross_sysroot_ready; then
    echo "[cross] Sysroot is current: ${CROSS_SYSROOT}"
    return 0
  fi

  local tmp="${CROSS_SYSROOT}.tmp.$$"
  rm -rf "${tmp}"
  mkdir -p "${tmp}" "$(dirname "${CROSS_SYSROOT}")"
  echo "[cross] Exporting Bianbu sysroot to: ${CROSS_SYSROOT}"
  if docker export "${container}" | tar -C "${tmp}" -xf -; then
    rm -rf "${tmp:?}/dev" "${tmp:?}/proc" "${tmp:?}/sys" "${tmp:?}/run" "${tmp:?}/tmp" 2>/dev/null || true
    rm -rf "${CROSS_SYSROOT}"
    mv "${tmp}" "${CROSS_SYSROOT}"
    printf '%s\n' "${fingerprint}" > "${stamp}"
  else
    rm -rf "${tmp}"
    return 1
  fi
}

cross_pkg_config_libdir() {
  printf '%s:%s:%s:%s\n' \
    "${CROSS_SYSROOT}/usr/lib/riscv64-linux-gnu/pkgconfig" \
    "${CROSS_SYSROOT}/usr/lib/pkgconfig" \
    "${CROSS_SYSROOT}/usr/share/pkgconfig" \
    "${CROSS_STAGING_PREFIX}/lib/pkgconfig"
}

cross_env_prefix_path() {
  printf '%s:%s:%s\n' \
    "${CROSS_STAGING_PREFIX}" \
    "${CROSS_SYSROOT}/usr" \
    "${CROSS_SYSROOT}/opt/ros/${ROS_DISTRO}"
}

cross_pythonpath() {
  local paths=()
  local preferred_ros_paths=()
  local host_paths=(
    "/usr/lib/python3/dist-packages"
    "/usr/local/lib/python3/dist-packages"
  )
  local remaining_target_paths=(
    "${CROSS_SYSROOT}/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages"
    "${CROSS_SYSROOT}/usr/lib/python3/dist-packages"
    "${CROSS_SYSROOT}/usr/local/lib/python3/dist-packages"
  )
  local path py_include py_version

  py_include="$(detect_cross_python_include)"
  py_version="${py_include##*/python}"
  if [[ "${py_version}" =~ ^[0-9]+\.[0-9]+$ ]]; then
    local ros_site_path ros_dist_path
    ros_site_path="${CROSS_SYSROOT}/opt/ros/${ROS_DISTRO}/lib/python${py_version}/site-packages"
    ros_dist_path="${CROSS_SYSROOT}/opt/ros/${ROS_DISTRO}/lib/python${py_version}/dist-packages"
    [[ -f "${ros_site_path}/em.py" ]] && preferred_ros_paths+=("${ros_site_path}")
    [[ -f "${ros_dist_path}/em.py" ]] && preferred_ros_paths+=("${ros_dist_path}")
  fi

  while IFS= read -r path; do
    [[ -n "${path}" ]] && remaining_target_paths+=("${path}")
  done < <(find "${CROSS_SYSROOT}/opt/ros/${ROS_DISTRO}/lib" "${CROSS_SYSROOT}/usr/lib" \( -name site-packages -o -name dist-packages \) -type d -maxdepth 2 -print 2>/dev/null | sort -V)

  declare -A seen=()
  for path in "${preferred_ros_paths[@]}" "${host_paths[@]}" "${remaining_target_paths[@]}"; do
    if [[ ( "${path}" == /usr/* || -d "${path}" ) && -z "${seen[${path}]:-}" ]]; then
      seen["${path}"]=1
      paths+=("${path}")
    fi
  done

  if [[ -n "${PYTHONPATH:-}" ]]; then
    paths+=("${PYTHONPATH}")
  fi

  (IFS=:; echo "${paths[*]}")
}

cross_rewrite_staging_runtime_paths() {
  [[ -n "${CROSS_STAGING_PREFIX:-}" && -n "${CROSS_SYSROOT:-}" ]] || return 0
  [[ -d "${CROSS_STAGING_PREFIX}" ]] || return 0

  local files=()
  local file parent_dir
  for file in \
    "${CROSS_STAGING_PREFIX}/setup.bash" \
    "${CROSS_STAGING_PREFIX}/setup.sh" \
    "${CROSS_STAGING_PREFIX}/setup.zsh"; do
    [[ -f "${file}" ]] && files+=("${file}")
  done

  parent_dir="${CROSS_STAGING_PREFIX}/share/ament_index/resource_index/parent_prefix_path"
  if [[ -d "${parent_dir}" ]]; then
    while IFS= read -r file; do
      [[ -n "${file}" ]] && files+=("${file}")
    done < <(find "${parent_dir}" -type f -print 2>/dev/null | sort)
  fi

  [[ ${#files[@]} -gt 0 ]] || return 0

  python3 - "${CROSS_SYSROOT}" "${CROSS_STAGING_PREFIX}" "${parent_dir}" "${files[@]}" <<'PY'
from pathlib import Path
import sys

sysroot = sys.argv[1].rstrip("/")
staging = sys.argv[2].rstrip("/")
parent_dir = sys.argv[3].rstrip("/")
if not sysroot:
    sys.exit(0)

for file_name in sys.argv[4:]:
    path = Path(file_name)
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        continue
    rewritten = text.replace(sysroot + "/", "/")
    path_text = str(path)
    if staging and parent_dir and (path_text == parent_dir or path_text.startswith(parent_dir + "/")):
        rewritten = rewritten.replace(staging, "{prefix}")
    if rewritten != text:
        path.write_text(rewritten, encoding="utf-8")
PY
}

cross_collect_absolute_sysroot_paths() {
  local roots=(
    "${CROSS_SYSROOT}/opt/ros/${ROS_DISTRO}"
    "${CROSS_SYSROOT}/usr/lib"
    "${CROSS_SYSROOT}/usr/share"
  )
  local root file path src
  declare -A seen=()

  for root in "${roots[@]}"; do
    [[ -d "${root}" ]] || continue
    while IFS= read -r -d '' file; do
      while IFS= read -r path; do
        [[ -n "${path}" ]] || continue
        path="${path%%[\\\")\}>]*}"
        [[ -n "${path}" ]] || continue
        [[ "${path}" == *'$'* || "${path}" == *'*'* ]] && continue
        [[ "${path}" == /usr/* ]] || continue
        [[ "${path}" == /usr/share/cmake-* ]] && continue
        case "${path}" in
          /usr/lib*/cmake|/usr/share/*/cmake)
            continue
            ;;
          /usr/lib*/cmake/*|/usr/share/*/cmake/*)
            continue
            ;;
        esac
        src="${CROSS_SYSROOT}${path}"
        [[ -e "${src}" ]] || continue
        [[ -n "${seen[${path}]:-}" ]] && continue
        seen["${path}"]=1
        printf '%s\n' "${path}"
      done < <(grep -hoE '/usr/[^";[:space:]]+' "${file}" 2>/dev/null || true)
    done < <(find "${root}" -type f -name '*.cmake' -print0 2>/dev/null)
  done
}

cross_bridge_absolute_sysroot_paths() {
  local container="$1"
  local bridge_args=()
  local path

  while IFS= read -r path; do
    [[ -n "${path}" ]] || continue
    bridge_args+=("${path}" "${CROSS_SYSROOT}${path}")
  done < <(cross_collect_absolute_sysroot_paths)

  docker exec "${container}" bash -lc '
    set -euo pipefail
    sysroot="$1"
    shift
    find /usr/lib /usr/share -type l \( -path "*/cmake" -o -path "*/cmake-"* \) -lname "${sysroot}/*" -delete 2>/dev/null || true
    find /usr/lib /usr/share -type l \( -path "*/cmake/*" -o -path "*/cmake-*/*" \) -lname "${sysroot}/*" -delete 2>/dev/null || true
    while [[ $# -gt 0 ]]; do
      target="$1"
      src="$2"
      shift 2
      [[ -e "${target}" ]] && continue
      mkdir -p "$(dirname "${target}")"
      ln -s "${src}" "${target}" 2>/dev/null || test -e "${target}"
    done
  ' _ "${CROSS_SYSROOT}" "${bridge_args[@]}"
}

detect_cross_rust_target() {
  local candidate
  while IFS= read -r candidate; do
    [[ -n "${candidate}" ]] || continue
    basename "${candidate}"
    return 0
  done < <(find "${CROSS_SYSROOT}/usr/lib" -path "*/rustlib/riscv64*-unknown-linux-gnu" -type d -print 2>/dev/null | sort -V)
  printf '%s\n' "riscv64gc-unknown-linux-gnu"
}

detect_cross_rust_sysroot() {
  local candidate
  while IFS= read -r candidate; do
    [[ -n "${candidate}" ]] || continue
    printf '%s\n' "${candidate%/lib/rustlib/*}"
    return 0
  done < <(find "${CROSS_SYSROOT}/usr/lib" -path "*/rustlib/riscv64*-unknown-linux-gnu" -type d -print 2>/dev/null | sort -V)
  printf '%s\n' "${CROSS_SYSROOT}/usr"
}

detect_cross_python_include() {
  find "${CROSS_SYSROOT}/usr/include" -maxdepth 1 -type d -name 'python3*' -print 2>/dev/null | sort -V | tail -n 1
}

detect_cross_python_library() {
  find "${CROSS_SYSROOT}/usr/lib" -type f -o -type l 2>/dev/null | \
    grep -E '/libpython3\.[0-9]+.*\.so$' | sort -V | tail -n 1 || true
}

detect_cross_numpy_include() {
  local candidate
  while IFS= read -r candidate; do
    [[ -n "${candidate}" ]] || continue
    dirname "$(dirname "${candidate}")"
    return 0
  done < <(find "${CROSS_SYSROOT}/usr/lib" -path '*/numpy/numpyconfig.h' -print 2>/dev/null | sort -V)
}

detect_cross_python_soabi() {
  local candidate soabi py_include py_version
  while IFS= read -r candidate; do
    [[ -n "${candidate}" ]] || continue
    soabi="$(basename "${candidate}")"
    soabi="${soabi#*.}"
    soabi="${soabi%.so}"
    [[ "${soabi}" == cpython-* ]] || continue
    printf '%s\n' "${soabi}"
    return 0
  done < <(find "${CROSS_SYSROOT}/usr/lib" -type f -name '*.cpython-*-riscv64-linux-gnu.so' -print 2>/dev/null | sort -V)

  py_include="$(detect_cross_python_include)"
  py_version="${py_include##*/python}"
  if [[ "${py_version}" =~ ^([0-9]+)\.([0-9]+)$ ]]; then
    printf 'cpython-%s%s-riscv64-linux-gnu\n' "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}"
  fi
}

write_cross_python_cmake_init() {
  local py_numpy_include numpy_config
  py_numpy_include="$(detect_cross_numpy_include)"
  mkdir -p "$(dirname "${CROSS_PYTHON_CMAKE_INIT}")"
  if [[ -z "${py_numpy_include}" ]]; then
    : > "${CROSS_PYTHON_CMAKE_INIT}"
    return 0
  fi
  numpy_config="${py_numpy_include}/numpy/numpyconfig.h"
  cat > "${CROSS_PYTHON_CMAKE_INIT}" <<EOF
if(CMAKE_CROSSCOMPILING AND EXISTS "${numpy_config}")
set(_numpy_h "${numpy_config}" CACHE FILEPATH "" FORCE)
include_directories(BEFORE SYSTEM "${py_numpy_include}")
endif()
EOF
}

write_toolchain_file() {
  mkdir -p "$(dirname "${CROSS_TOOLCHAIN_FILE}")"
  cat > "${CROSS_TOOLCHAIN_FILE}" <<EOF
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR riscv64)
set(CMAKE_SYSROOT "${CROSS_SYSROOT}")
set(CMAKE_C_COMPILER /usr/bin/riscv64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/riscv64-linux-gnu-g++)
set(CMAKE_AR /usr/bin/riscv64-linux-gnu-ar)
set(CMAKE_RANLIB /usr/bin/riscv64-linux-gnu-ranlib)
set(CMAKE_FIND_ROOT_PATH "${CROSS_SYSROOT}" "${CROSS_STAGING_PREFIX}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
EOF
}

write_meson_cross_file() {
  mkdir -p "$(dirname "${CROSS_MESON_CROSS_FILE}")"
  cat > "${CROSS_MESON_CROSS_FILE}" <<EOF
[binaries]
c = '/usr/bin/riscv64-linux-gnu-gcc'
cpp = '/usr/bin/riscv64-linux-gnu-g++'
ar = '/usr/bin/riscv64-linux-gnu-ar'
strip = '/usr/bin/riscv64-linux-gnu-strip'
pkgconfig = '/usr/bin/pkg-config'

[properties]
sys_root = '${CROSS_SYSROOT}'
pkg_config_libdir = '${CROSS_SYSROOT}/usr/lib/riscv64-linux-gnu/pkgconfig:${CROSS_SYSROOT}/usr/lib/pkgconfig:${CROSS_SYSROOT}/usr/share/pkgconfig:${CROSS_STAGING_PREFIX}/lib/pkgconfig'
c_args = ['--sysroot=${CROSS_SYSROOT}']
cpp_args = ['--sysroot=${CROSS_SYSROOT}']
c_link_args = ['--sysroot=${CROSS_SYSROOT}', '-Wl,-rpath-link,${CROSS_STAGING_PREFIX}/lib', '-Wl,-rpath-link,${CROSS_SYSROOT}/usr/lib/riscv64-linux-gnu', '-Wl,-rpath-link,${CROSS_SYSROOT}/usr/lib']
cpp_link_args = ['--sysroot=${CROSS_SYSROOT}', '-Wl,-rpath-link,${CROSS_STAGING_PREFIX}/lib', '-Wl,-rpath-link,${CROSS_SYSROOT}/usr/lib/riscv64-linux-gnu', '-Wl,-rpath-link,${CROSS_SYSROOT}/usr/lib']

[host_machine]
system = 'linux'
cpu_family = 'riscv64'
cpu = 'riscv64'
endian = 'little'
EOF
}

cross_ros2_cmake_extra_args() {
  printf '%s\n' "-DCMAKE_PROJECT_INCLUDE=${CROSS_PYTHON_CMAKE_INIT}"
}

cross_cmake_extra_args() {
  local rust_target rust_sysroot py_include py_library py_numpy_include py_soabi
  rust_target="$(detect_cross_rust_target)"
  rust_sysroot="$(detect_cross_rust_sysroot)"
  py_include="$(detect_cross_python_include)"
  py_library="$(detect_cross_python_library)"
  py_numpy_include="$(detect_cross_numpy_include)"
  py_soabi="$(detect_cross_python_soabi)"

  printf '%s\n' \
    "-DCMAKE_TOOLCHAIN_FILE=${CROSS_TOOLCHAIN_FILE}" \
    "-DCMAKE_SYSTEM_PROCESSOR=riscv64" \
    "-DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config" \
    "-DCMAKE_MAKE_PROGRAM=/usr/bin/make" \
    "-DPython3_EXECUTABLE=/usr/bin/python3" \
    "-DPython3_ROOT_DIR=/usr" \
    "-DPython3_FIND_STRATEGY=LOCATION" \
    "-DPYTHON_EXECUTABLE=/usr/bin/python3" \
    "-DBUILD_TESTING=OFF" \
    "-DCMAKE_DISABLE_FIND_PACKAGE_rosidl_generator_rs=TRUE" \
    "-DCMAKE_PREFIX_PATH=${CROSS_STAGING_PREFIX};${CROSS_SYSROOT}/usr;${CROSS_SYSROOT}/usr/lib/riscv64-linux-gnu/cmake;${CROSS_SYSROOT}/opt/ros/${ROS_DISTRO}" \
    "-DCMAKE_FIND_ROOT_PATH=${CROSS_SYSROOT};${CROSS_STAGING_PREFIX}" \
    "-DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER" \
    "-DCMAKE_IGNORE_PATH=${CROSS_SYSROOT}/bin;${CROSS_SYSROOT}/sbin;${CROSS_SYSROOT}/usr/bin;${CROSS_SYSROOT}/usr/sbin;${CROSS_SYSROOT}/usr/local/bin;${CROSS_SYSROOT}/usr/local/sbin" \
    "-DCMAKE_EXE_LINKER_FLAGS=-Wl,--allow-shlib-undefined -Wl,-rpath-link,${CROSS_STAGING_PREFIX}/lib -Wl,-rpath-link,${CROSS_SYSROOT}/usr/lib/riscv64-linux-gnu -Wl,-rpath-link,${CROSS_SYSROOT}/usr/lib" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-Wl,-rpath-link,${CROSS_STAGING_PREFIX}/lib -Wl,-rpath-link,${CROSS_SYSROOT}/usr/lib/riscv64-linux-gnu -Wl,-rpath-link,${CROSS_SYSROOT}/usr/lib" \
    "-DGIT_EXECUTABLE=/usr/bin/git" \
    "-DMESON_EXE=/usr/bin/meson" \
    "-DNINJA_EXE=/usr/bin/ninja" \
    "-DPROTOC_EXE=/usr/bin/protoc" \
    "-DGRPC_CPP_PLUGIN_EXE=/usr/bin/grpc_cpp_plugin" \
    "-DSROBOTIS_QT_MOC_EXECUTABLE=/usr/lib/qt5/bin/moc" \
    "-DSROBOTIS_QT_UIC_EXECUTABLE=/usr/lib/qt5/bin/uic" \
    "-DSROBOTIS_QT_RCC_EXECUTABLE=/usr/lib/qt5/bin/rcc" \
    "-DSROBOTIS_MESON_CROSS_FILE=${CROSS_MESON_CROSS_FILE}" \
    "-DOpenCV_DIR=${CROSS_SYSROOT}/opt/opencv-spacemit/lib/cmake/opencv4" \
    "-DOpenCV_INSTALL_DIR=${CROSS_SYSROOT}/opt/opencv-spacemit" \
    "-DSPACEMIT_DIR=${CROSS_SYSROOT}/usr" \
    "-DCMAKE_PROGRAM_PATH=${CROSS_HOST_PREFIX}/bin" \
    "-DCMAKE_LIBRARY_PATH=${CROSS_HOST_PREFIX}/lib;${CROSS_HOST_PREFIX}/lib/x86_64-linux-gnu" \
    "-DCARGO_EXECUTABLE=${CROSS_CARGO_EXECUTABLE}" \
    "-DSROBOTIS_CARGO_HOME=${CROSS_OUTPUT_ROOT}/.cargo" \
    "-DSROBOTIS_CARGO_RUSTC=${CROSS_RUSTC_EXECUTABLE}" \
    "-DSROBOTIS_CARGO_TARGET=${rust_target}" \
    "-DSROBOTIS_CARGO_TARGET_LINKER=/usr/bin/riscv64-linux-gnu-gcc" \
    "-DSROBOTIS_CARGO_TARGET_AR=/usr/bin/riscv64-linux-gnu-ar" \
    "-DSROBOTIS_RUST_SYSROOT=${rust_sysroot}" \
    "-DSROBOTIS_CARGO_LINK_SYSROOT=${CROSS_SYSROOT}"

  if [[ "${SROBOTIS_CROSS_DISABLE_ROSIDL_GENERATOR_PY:-0}" == "1" ]]; then
    printf '%s\n' "-DCMAKE_DISABLE_FIND_PACKAGE_rosidl_generator_py=TRUE"
  fi
  if [[ -n "${py_include}" ]]; then
    printf '%s\n' "-DPython3_INCLUDE_DIR=${py_include}" "-DPYTHON_INCLUDE_DIR=${py_include}"
    if [[ -n "${py_numpy_include}" ]]; then
      printf '%s\n' \
        "-DPython3_NumPy_INCLUDE_DIR=${py_numpy_include}" \
        "-DPython3_NumPy_INCLUDE_DIRS=${py_numpy_include}" \
        "-DPYTHON_INCLUDE_DIRS=${py_include};${py_numpy_include}" \
        "-DPythonExtra_INCLUDE_DIRS=${py_include};${py_numpy_include}"
    fi
  fi
  if [[ -n "${py_library}" ]]; then
    printf '%s\n' "-DPython3_LIBRARY=${py_library}" "-DPYTHON_LIBRARY=${py_library}" "-DPYTHON_LIBRARIES=${py_library}"
  fi
  if [[ -n "${py_soabi}" ]]; then
    printf '%s\n' "-DPython3_SOABI=${py_soabi}" "-DPYTHON_SOABI=${py_soabi}"
  fi
  if [[ -n "${SROBOTIS_CMAKE_EXTRA_ARGS:-}" ]]; then
    while IFS= read -r line; do
      [[ -n "${line}" ]] && printf '%s\n' "${line}"
    done <<< "${SROBOTIS_CMAKE_EXTRA_ARGS}"
  fi
}

cross_chown_output() {
  local container="$1"
  [[ "${SROBOTIS_CROSS_FIX_OUTPUT_OWNER:-1}" == "1" ]] || return 0
  docker exec "${container}" chown -R "$(id -u):$(id -g)" "${CROSS_OUTPUT_ROOT}" >/dev/null 2>&1 || true
}

run_build_in_ubuntu() {
  local container="$1"
  shift || true
  local cmake_args pkg_config_libdir pythonpath env_prefix_path
  local ros2_cmake_args
  cmake_args="$(cross_cmake_extra_args)"
  ros2_cmake_args="$(cross_ros2_cmake_extra_args)"
  pkg_config_libdir="$(cross_pkg_config_libdir)"
  pythonpath="$(cross_pythonpath)"
  env_prefix_path="$(cross_env_prefix_path)"

  cross_bridge_absolute_sysroot_paths "${container}"

  echo "[cross] Running build in Ubuntu host container: $*"
  set +e
  docker exec -i \
    --workdir "${REPO_ROOT}" \
    -e "HOME=/root" \
    -e "REPO_ROOT=${REPO_ROOT}" \
    -e "OUTPUT_ROOT=${CROSS_OUTPUT_ROOT}" \
    -e "STAGING_PREFIX=${CROSS_STAGING_PREFIX}" \
    -e "ROOTFS_PREFIX=${CROSS_ROOTFS_PREFIX}" \
    -e "PREFIX=${CROSS_STAGING_PREFIX}" \
    -e "PARALLEL_JOBS=${PARALLEL_JOBS}" \
    -e "SROBOTIS_PARALLEL_JOBS_EXPLICIT=${SROBOTIS_PARALLEL_JOBS_EXPLICIT:-0}" \
    -e "BUILD_TARGET=${BUILD_TARGET:-}" \
    -e "BUILD_TARGET_FILE=${BUILD_TARGET_FILE:-}" \
    -e "ROS_DISTRO=${ROS_DISTRO}" \
    -e "ROS_SETUP=${ROS_SETUP}" \
    -e "SROBOTIS_CROSS_BUILD=1" \
    -e "SROBOTIS_SKIP_DEPS_CHECK=1" \
    -e "SDK_BUILD_ARCH=${CROSS_TARGET_ARCH}" \
    -e "PKG_CONFIG_SYSROOT_DIR=${CROSS_SYSROOT}" \
    -e "PKG_CONFIG_LIBDIR=${pkg_config_libdir}" \
    -e "PYTHONPATH=${pythonpath}" \
    -e "AMENT_PREFIX_PATH=${env_prefix_path}" \
    -e "CMAKE_PREFIX_PATH=${env_prefix_path}" \
    -e "SROBOTIS_CMAKE_EXTRA_ARGS=${cmake_args}" \
    -e "SROBOTIS_ROS2_CMAKE_EXTRA_ARGS=${ros2_cmake_args}" \
    -e "SROBOTIS_CROSS_SYSROOT=${CROSS_SYSROOT}" \
    -e "SROBOTIS_CROSS_OUTPUT_ROOT=${CROSS_OUTPUT_ROOT}" \
    -e "SROBOTIS_CROSS_STAGING=${CROSS_STAGING_PREFIX}" \
    -e "SROBOTIS_CROSS_ROOTFS=${CROSS_ROOTFS_PREFIX}" \
    -e "SROBOTIS_CROSS_HOST_PREFIX=${CROSS_HOST_PREFIX}" \
    -e "SROBOTIS_CROSS_TOOLCHAIN_FILE=${CROSS_TOOLCHAIN_FILE}" \
  "${container}" \
    ./build/build.sh "$@"
  local rc=$?
  set -e
  cross_chown_output "${container}"
  if [[ "${rc}" -eq 0 ]]; then
    cross_rewrite_staging_runtime_paths || rc=$?
  fi
  return "${rc}"
}

cross_prepare_and_build() {
  local cmd="$1"
  shift || true
  local payload=()
  while [[ $# -gt 0 && "$1" != "--" ]]; do
    payload+=("$1")
    shift
  done
  if [[ "${1:-}" == "--" ]]; then
    shift
  fi
  local original_args=("$@")

  load_build_config
  if [[ -z "${BUILD_CONFIG_FILE:-}" || ! -f "${BUILD_CONFIG_FILE}" ]]; then
    cross_error "${cmd} requires BUILD_TARGET or BUILD_TARGET_FILE"
    return 1
  fi
  select_cross_images
  init_cross_paths

  ensure_docker_available
  ensure_docker_image "${CROSS_UBUNTU_IMAGE}"
  ensure_cross_bianbu_image

  local ubuntu_container bianbu_container
  ubuntu_container="$(cross_container_name "ubuntu")"
  bianbu_container="$(cross_container_name "bianbu")"

  ensure_container "${ubuntu_container}" "${CROSS_UBUNTU_IMAGE}" "${SROBOTIS_CROSS_UBUNTU_PLATFORM:-}"
  ensure_container "${bianbu_container}" "${CROSS_BIANBU_IMAGE}" "${SROBOTIS_CROSS_BIANBU_PLATFORM:-linux/riscv64}"
  configure_bianbu_container "${bianbu_container}"

  split_cross_dependencies "${cmd}" "${payload[@]}"
  print_cross_deps_split
  install_deps_in_container "${ubuntu_container}" "Ubuntu host" "${HOST_DEP_LINES[@]}"
  install_deps_in_container "${bianbu_container}" "Bianbu sysroot" "${SYSROOT_DEP_LINES[@]}"

  local fingerprint
  fingerprint="$(cross_sysroot_deps_fingerprint)"
  sync_bianbu_sysroot "${bianbu_container}" "${SROBOTIS_CROSS_REFRESH_SYSROOT:-0}" "${fingerprint}"
  write_toolchain_file
  write_cross_python_cmake_init
  write_meson_cross_file

  run_build_in_ubuntu "${ubuntu_container}" "${original_args[@]}"
}

cross_clean() {
  local clean_type="${1:-all}"
  load_build_config || true
  init_cross_paths
  case "${clean_type}" in
    all)
      rm -rf "${CROSS_OUTPUT_ROOT}"
      ;;
    cmake|C)
      rm -rf "${CROSS_OUTPUT_ROOT}/build/cmake"
      ;;
    ros2|R)
      rm -rf "${CROSS_OUTPUT_ROOT}/build/ros2" "${CROSS_OUTPUT_ROOT}/log/ros2"
      ;;
    *)
      cross_error "Unknown clean type: ${clean_type}"
      return 1
      ;;
  esac
  echo "[cross] Cleaned ${clean_type}: ${CROSS_OUTPUT_ROOT}"
}

cross_deploy_rootfs() {
  load_build_config
  init_cross_paths
  cross_rewrite_staging_runtime_paths
  OUTPUT_ROOT="${CROSS_OUTPUT_ROOT}" \
    STAGING_PREFIX="${CROSS_STAGING_PREFIX}" \
    ROOTFS_PREFIX="${CROSS_ROOTFS_PREFIX}" \
    PREFIX="${CROSS_STAGING_PREFIX}" \
    "${REPO_ROOT}/build/build.sh" deploy-rootfs
}

cross_runtime_display_path() {
  local path="$1"
  if [[ "${path}" == "${CROSS_ROOTFS_PREFIX}/"* ]]; then
    printf 'rootfs/%s\n' "${path#"${CROSS_ROOTFS_PREFIX}/"}"
  elif [[ "${path}" == "${CROSS_STAGING_PREFIX}/"* ]]; then
    printf 'staging/%s\n' "${path#"${CROSS_STAGING_PREFIX}/"}"
  elif [[ "${path}" == "${CROSS_SYSROOT}/"* ]]; then
    printf '/%s\n' "${path#"${CROSS_SYSROOT}/"}"
  else
    printf '%s\n' "${path}"
  fi
}

cross_runtime_is_elf() {
  local path="$1"
  [[ -f "${path}" || -L "${path}" ]] || return 1
  LC_ALL=C readelf -h "${path}" >/dev/null 2>&1
}

cross_runtime_scan_root() {
  if [[ -d "${CROSS_ROOTFS_PREFIX}" ]] && find "${CROSS_ROOTFS_PREFIX}" -mindepth 1 -print -quit 2>/dev/null | grep -q .; then
    printf '%s\n' "${CROSS_ROOTFS_PREFIX}"
    return 0
  fi
  if [[ -d "${CROSS_STAGING_PREFIX}" ]] && find "${CROSS_STAGING_PREFIX}" -mindepth 1 -print -quit 2>/dev/null | grep -q .; then
    printf '%s\n' "${CROSS_STAGING_PREFIX}"
    return 0
  fi
  return 1
}

cross_runtime_collect_elfs() {
  local root="$1"
  local file
  [[ -d "${root}" ]] || return 0
  while IFS= read -r -d '' file; do
    if cross_runtime_is_elf "${file}"; then
      printf '%s\n' "${file}"
    fi
  done < <(find "${root}" \( -type f -o -type l \) -print0 2>/dev/null)
}

cross_runtime_read_needed() {
  local elf="$1"
  LC_ALL=C readelf -d "${elf}" 2>/dev/null | sed -n 's/.*Shared library: \[\(.*\)\].*/\1/p' || true
}

cross_runtime_read_runpaths() {
  local elf="$1"
  LC_ALL=C readelf -d "${elf}" 2>/dev/null | \
    sed -n 's/.*Library .*path: \[\(.*\)\].*/\1/p' | \
    tr ':' '\n' || true
}

cross_runtime_expand_search_path() {
  local elf="$1"
  local entry="$2"
  local origin
  origin="$(dirname "${elf}")"
  entry="${entry//\$\{ORIGIN\}/${origin}}"
  entry="${entry//\$ORIGIN/${origin}}"

  if [[ "${entry}" == "${CROSS_ROOTFS_PREFIX}"/* || "${entry}" == "${CROSS_STAGING_PREFIX}"/* || "${entry}" == "${CROSS_SYSROOT}"/* ]]; then
    realpath -m "${entry}"
    return 0
  fi

  if [[ "${entry}" == /* ]]; then
    local prefix
    for prefix in "${CROSS_ROOTFS_PREFIX}" "${CROSS_STAGING_PREFIX}" "${CROSS_SYSROOT}"; do
      if [[ -d "${prefix}${entry}" ]]; then
        realpath -m "${prefix}${entry}"
      fi
    done
    return 0
  fi

  realpath -m "${origin}/${entry}"
}

cross_runtime_default_lib_dirs() {
  local dirs=(
    "${CROSS_ROOTFS_PREFIX}/lib"
    "${CROSS_ROOTFS_PREFIX}/usr/lib/riscv64-linux-gnu"
    "${CROSS_ROOTFS_PREFIX}/usr/lib"
    "${CROSS_STAGING_PREFIX}/lib"
    "${CROSS_STAGING_PREFIX}/usr/lib/riscv64-linux-gnu"
    "${CROSS_STAGING_PREFIX}/usr/lib"
    "${CROSS_SYSROOT}/lib/riscv64-linux-gnu"
    "${CROSS_SYSROOT}/lib"
    "${CROSS_SYSROOT}/usr/lib/riscv64-linux-gnu"
    "${CROSS_SYSROOT}/usr/lib"
    "${CROSS_SYSROOT}/usr/local/lib"
    "${CROSS_SYSROOT}/opt/ros/${ROS_DISTRO}/lib"
    "${CROSS_SYSROOT}/opt/opencv-spacemit/lib"
  )
  local dir
  for dir in "${dirs[@]}"; do
    [[ -d "${dir}" ]] && printf '%s\n' "${dir}"
  done
  if [[ -d "${CROSS_SYSROOT}/opt" ]]; then
    find "${CROSS_SYSROOT}/opt" -maxdepth 3 -type d -name lib -print 2>/dev/null || true
  fi
}

cross_runtime_resolve_lib() {
  local needed="$1"
  local elf="$2"
  local dir candidate
  declare -A seen_dirs=()

  while IFS= read -r dir; do
    [[ -n "${dir}" ]] || continue
    if [[ -z "${seen_dirs[${dir}]:-}" ]]; then
      seen_dirs["${dir}"]=1
      candidate="${dir}/${needed}"
      if [[ -e "${candidate}" ]]; then
        realpath -m "${candidate}"
        return 0
      fi
    fi
  done < <(
    while IFS= read -r dir; do
      [[ -n "${dir}" ]] && cross_runtime_expand_search_path "${elf}" "${dir}"
    done < <(cross_runtime_read_runpaths "${elf}")
    cross_runtime_default_lib_dirs
  )

  return 1
}

cross_runtime_sysroot_relpath() {
  local path="$1"
  [[ "${path}" == "${CROSS_SYSROOT}/"* ]] || return 1
  printf '/%s\n' "${path#"${CROSS_SYSROOT}/"}"
}

cross_runtime_dpkg_owner() {
  local relpath="$1"
  local line owners owner
  line="$(dpkg-query --admindir="${CROSS_SYSROOT}/var/lib/dpkg" -S "${relpath}" 2>/dev/null | head -n 1 || true)"
  [[ -n "${line}" ]] || return 1
  owners="${line%: "${relpath}"}"
  owner="${owners%%, *}"
  [[ -n "${owner}" ]] || return 1
  printf '%s\n' "${owner}"
}

cross_runtime_apt_name() {
  local owner="$1"
  printf '%s\n' "${owner%%:*}"
}

cross_runtime_pkg_version() {
  local owner="$1"
  local apt_name
  apt_name="$(cross_runtime_apt_name "${owner}")"
  dpkg-query --admindir="${CROSS_SYSROOT}/var/lib/dpkg" -W -f='${Version}' "${owner}" 2>/dev/null || \
    dpkg-query --admindir="${CROSS_SYSROOT}/var/lib/dpkg" -W -f='${Version}' "${apt_name}" 2>/dev/null || true
}

cross_runtime_is_base_pkg() {
  local apt_name="$1"
  case "${apt_name}" in
    base-files|base-passwd|gcc-*-base|libc6|libgcc-s1|libstdc++6|libatomic1)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

cross_runtime_add_detail() {
  local -n details_ref="$1"
  local -n seen_ref="$2"
  local key="$3"
  local detail="$4"
  local pkg_key="${key%%"${CROSS_DEP_SEP}"*}"
  if [[ -n "${seen_ref[${key}]:-}" ]]; then
    return 0
  fi
  seen_ref["${key}"]=1
  if [[ -n "${details_ref[${pkg_key}]:-}" ]]; then
    details_ref["${pkg_key}"]+=$'\n'
  fi
  details_ref["${pkg_key}"]+="${detail}"
}

cross_runtime_print_pkg_section() {
  local title="$1"
  local -n versions_ref="$2"
  # shellcheck disable=SC2178
  local -n details_ref="$3"
  local key version
  echo "${title}"
  if [[ ${#versions_ref[@]} -eq 0 ]]; then
    echo "  (none)"
    return 0
  fi
  while IFS= read -r key; do
    [[ -n "${key}" ]] || continue
    version="${versions_ref[${key}]:-}"
    if [[ -n "${version}" ]]; then
      printf '  - %s (= %s)\n' "${key}" "${version}"
    else
      printf '  - %s\n' "${key}"
    fi
    while IFS= read -r detail; do
      [[ -n "${detail}" ]] && printf '      %s\n' "${detail}"
    done <<< "${details_ref[${key}]:-}"
  done < <(printf '%s\n' "${!versions_ref[@]}" | sort)
}

cross_runtime_print_text_section() {
  local title="$1"
  local -n lines_ref="$2"
  local key
  echo "${title}"
  if [[ ${#lines_ref[@]} -eq 0 ]]; then
    echo "  (none)"
    return 0
  fi
  while IFS= read -r key; do
    [[ -n "${key}" ]] && printf '  - %s\n' "${key}"
  done < <(printf '%s\n' "${!lines_ref[@]}" | sort)
}

cross_runtime_print_install_command() {
  local -n versions_ref="$1"
  local packages=()
  local key
  while IFS= read -r key; do
    [[ -n "${key}" ]] && packages+=("${key}")
  done < <(printf '%s\n' "${!versions_ref[@]}" | sort)

  echo "[cross] Install command:"
  if [[ ${#packages[@]} -eq 0 ]]; then
    echo "  (none)"
  else
    printf 'apt-get install -y'
    printf ' %s' "${packages[@]}"
    printf '\n'
  fi
}

cross_runtime_deps() {
  local strict=0
  local include_base=0
  local scope="${1:-all}"
  shift || true

  while [[ "${scope}" == --* ]]; do
    case "${scope}" in
      --strict)
        strict=1
        ;;
      --include-base)
        include_base=1
        ;;
      *)
        cross_error "Unknown runtime-deps option: ${scope}"
        return 1
        ;;
    esac
    scope="${1:-all}"
    shift || true
  done

  case "${scope}" in
    ""|all)
      ;;
    package)
      local pkg_key
      pkg_key="$(cross_pkg_key_from_arg "${1:-}")"
      [[ -n "${pkg_key}" ]] || { cross_error "runtime-deps package requires a package path"; return 1; }
      cross_validate_package_path "${pkg_key}" || return 1
      ;;
    *)
      cross_error "Unknown runtime-deps scope: ${scope}"
      return 1
      ;;
  esac

  load_build_config
  if [[ -z "${BUILD_CONFIG_FILE:-}" || ! -f "${BUILD_CONFIG_FILE}" ]]; then
    cross_error "runtime-deps requires BUILD_TARGET or BUILD_TARGET_FILE"
    return 1
  fi
  init_cross_paths

  local scan_root
  if ! scan_root="$(cross_runtime_scan_root)"; then
    cross_error "No cross rootfs/staging output found under ${CROSS_OUTPUT_ROOT}; build first"
    return 1
  fi

  local elfs=()
  mapfile -t elfs < <(cross_runtime_collect_elfs "${scan_root}")
  echo "[cross] Runtime dependency scan root: $(cross_runtime_display_path "${scan_root}")"

  if [[ ${#elfs[@]} -eq 0 ]]; then
    echo "[cross] No ELF files found"
    return 0
  fi

  declare -A apt_versions=()
  # shellcheck disable=SC2034
  declare -A apt_details=()
  # shellcheck disable=SC2034
  declare -A apt_detail_seen=()
  declare -A base_versions=()
  # shellcheck disable=SC2034
  declare -A base_details=()
  # shellcheck disable=SC2034
  declare -A base_detail_seen=()
  # shellcheck disable=SC2034
  declare -A sdk_lines=()
  declare -A unresolved_lines=()

  local elf needed resolved display request_rel relpath owner apt_name version detail detail_key
  for elf in "${elfs[@]}"; do
    request_rel="$(cross_runtime_display_path "${elf}")"
    while IFS= read -r needed; do
      [[ -n "${needed}" ]] || continue
      if ! resolved="$(cross_runtime_resolve_lib "${needed}" "${elf}")"; then
        unresolved_lines["${needed} required by ${request_rel}"]=1
        continue
      fi

      display="$(cross_runtime_display_path "${resolved}")"
      if [[ "${resolved}" == "${CROSS_ROOTFS_PREFIX}/"* || "${resolved}" == "${CROSS_STAGING_PREFIX}/"* ]]; then
        # shellcheck disable=SC2034
        sdk_lines["${needed} => ${display} (required by ${request_rel})"]=1
        continue
      fi

      if [[ "${resolved}" != "${CROSS_SYSROOT}/"* ]]; then
        unresolved_lines["${needed} resolved outside cross output/sysroot: ${display} (required by ${request_rel})"]=1
        continue
      fi

      relpath="$(cross_runtime_sysroot_relpath "${resolved}")" || {
        unresolved_lines["${needed} => ${display} has no sysroot-relative path (required by ${request_rel})"]=1
        continue
      }
      if ! owner="$(cross_runtime_dpkg_owner "${relpath}")"; then
        unresolved_lines["${needed} => ${display} has no dpkg owner in sysroot (required by ${request_rel})"]=1
        continue
      fi

      apt_name="$(cross_runtime_apt_name "${owner}")"
      version="$(cross_runtime_pkg_version "${owner}")"
      detail="${needed} => ${display} (required by ${request_rel})"
      detail_key="${apt_name}${CROSS_DEP_SEP}${detail}"
      if cross_runtime_is_base_pkg "${apt_name}" && [[ "${include_base}" != "1" ]]; then
        # shellcheck disable=SC2034
        base_versions["${apt_name}"]="${version}"
        cross_runtime_add_detail base_details base_detail_seen "${detail_key}" "${detail}"
      else
        # shellcheck disable=SC2034
        apt_versions["${apt_name}"]="${version}"
        cross_runtime_add_detail apt_details apt_detail_seen "${detail_key}" "${detail}"
      fi
    done < <(cross_runtime_read_needed "${elf}")
  done

  cross_runtime_print_pkg_section "[cross] Board apt runtime dependencies:" apt_versions apt_details
  cross_runtime_print_install_command apt_versions
  cross_runtime_print_text_section "[cross] SDK-bundled runtime libraries:" sdk_lines
  cross_runtime_print_pkg_section "[cross] Base/system runtime libraries:" base_versions base_details
  cross_runtime_print_text_section "[cross] Unresolved runtime libraries:" unresolved_lines

  if [[ "${strict}" == "1" && ${#unresolved_lines[@]} -gt 0 ]]; then
    return 1
  fi
}

CROSS_PARSED_CMD=""
CROSS_PARSED_PAYLOAD=()

cross_parse_args() {
  CROSS_PARSED_CMD="help"
  CROSS_PARSED_PAYLOAD=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -j|-j*|-v|--log=*|--py|-py|--)
        if [[ "$1" == "-j" ]]; then
          shift 2 || true
        else
          shift
        fi
        ;;
      -*)
        break
        ;;
      *)
        break
        ;;
    esac
  done

  CROSS_PARSED_CMD="${1:-help}"
  if [[ $# -gt 0 ]]; then
    shift
    CROSS_PARSED_PAYLOAD=("$@")
  fi
}

main() {
  local original_args=("$@")
  cross_parse_args "$@"
  local cmd="${CROSS_PARSED_CMD}"

  case "${cmd}" in
    deps)
      split_cross_dependencies "deps" "${CROSS_PARSED_PAYLOAD[@]}"
      print_cross_deps_split
      ;;
    all|cmake|C|ros2|R|package|pkg)
      cross_prepare_and_build "${cmd}" "${CROSS_PARSED_PAYLOAD[@]}" -- "${original_args[@]}"
      ;;
    clean)
      cross_clean "${CROSS_PARSED_PAYLOAD[0]:-all}"
      ;;
    deploy-rootfs|rootfs)
      cross_deploy_rootfs
      ;;
    runtime-deps|runtime-depends|rtdeps)
      cross_runtime_deps "${CROSS_PARSED_PAYLOAD[@]}"
      ;;
    help|--help|-h)
      cross_usage
      ;;
    *)
      cross_error "Unknown command: ${cmd}"
      cross_usage
      return 1
      ;;
  esac
}

if [[ "${SROBOTIS_CROSS_BUILD_SH_NO_MAIN:-0}" != "1" ]]; then
  main "$@"
fi
