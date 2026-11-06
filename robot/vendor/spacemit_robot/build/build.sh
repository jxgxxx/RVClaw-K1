#!/usr/bin/env bash
#
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#

set -euo pipefail

# SDK root = one level above this script
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_ROOT="${OUTPUT_ROOT:-${REPO_ROOT}/output}"

# Prefix/layout defaults (can be overridden by env)
CM_BUILD_ROOT="${OUTPUT_ROOT}/build/cmake"
ROS2_BUILD_ROOT="${OUTPUT_ROOT}/build/ros2"
ROS2_LOG_ROOT="${OUTPUT_ROOT}/log/ros2"

STAGING_PREFIX="${STAGING_PREFIX:-${OUTPUT_ROOT}/staging}"
ROOTFS_PREFIX="${ROOTFS_PREFIX:-${OUTPUT_ROOT}/rootfs}"
PREFIX="${PREFIX:-${STAGING_PREFIX}}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"

PARALLEL_JOBS="${PARALLEL_JOBS:-$(nproc)}"

BUILD_TARGET="${BUILD_TARGET:-}"
BUILD_TARGET_FILE="${BUILD_TARGET_FILE:-}"

# Load modular implementations
source "${REPO_ROOT}/build/common.sh"
source "${REPO_ROOT}/build/nonros2.sh"
source "${REPO_ROOT}/build/ros2.sh"
source "${REPO_ROOT}/build/docker_build.sh"

# ============================================================================
# Single package build functions
# ============================================================================

# Build single package (supports custom scripts)
build_single_package() {
  local pkg_dir="$1"

  [[ ! -d "${pkg_dir}" ]] && { echo "[build] ERROR: Package directory not found: ${pkg_dir}" >&2; exit 1; }

  # Normalize to absolute path
  pkg_dir="$(cd "${pkg_dir}" && pwd)"
  if [[ "${pkg_dir}" != "${REPO_ROOT}/"* ]]; then
    echo "[build] ERROR: Package directory must be inside repo: ${pkg_dir}" >&2
    exit 1
  fi
  local pkg_key="${pkg_dir#"${REPO_ROOT}"/}"

  # Check for custom build script first
  if [[ -f "${pkg_dir}/build.sh" ]] && [[ -x "${pkg_dir}/build.sh" ]]; then
    build_one_nonros2_pkg "${pkg_key}" "${_want_python_wheels:-0}"
    return $?
  fi

  # If package has package.xml, use <export><build_type> to decide ROS2 vs CMake.
  # ament_cmake/ament_python → ROS2 build; cmake or missing → fall through to CMake.
  if [[ -f "${pkg_dir}/package.xml" ]]; then
    local build_type
    build_type="$(get_package_build_type "${pkg_dir}" || true)"
    if [[ "${build_type}" == "ament_cmake" || "${build_type}" == "ament_python" ]]; then
      load_ros2_env
      local pkg_name
      pkg_name="$(get_ros2_package_name "${pkg_dir}")"
      if [[ "${pkg_dir}" == "${REPO_ROOT}/middleware/ros2/"* ]]; then
        build_ros2_middleware "${pkg_name}"
        return $?
      elif [[ "${pkg_dir}" == "${REPO_ROOT}/application/ros2/"* ]]; then
        build_ros2_applications "${pkg_name}"
        return $?
      else
        echo "[build] ERROR: ROS2 package (build_type=${build_type}) must be under middleware/ros2/ or application/ros2/" >&2
        exit 1
      fi
    fi
    # build_type is cmake or empty → treat as non-ROS2, fall through to CMake
  fi

  # CMake package
  if [[ -f "${pkg_dir}/CMakeLists.txt" ]]; then
    # Packages under application/native need to build entire workspace
    if [[ "${pkg_dir}" == "${REPO_ROOT}/application/native/"* ]]; then
      # Native apps are treated as a regular non-ROS2 CMake package in this build system.
      build_one_nonros2_pkg "${pkg_key}" "${_want_python_wheels:-0}"
    else
      build_one_nonros2_pkg "${pkg_key}" "${_want_python_wheels:-0}"
    fi
    return $?
  fi

  echo "[build] ERROR: No valid package identifier found (CMakeLists.txt, package.xml, or build.sh)" >&2
  exit 1
}

# Clean single package
clean_single_package() {
  local pkg_dir="$1"

  [[ ! -d "${pkg_dir}" ]] && { echo "[build] ERROR: Package directory not found: ${pkg_dir}" >&2; exit 1; }

  # ROS2 package cleanup (only for ament_cmake/ament_python; cmake build_type uses CMake path below)
  if [[ -f "${pkg_dir}/package.xml" ]]; then
    local build_type
    build_type="$(get_package_build_type "${pkg_dir}" || true)"
    if [[ "${build_type}" == "ament_cmake" || "${build_type}" == "ament_python" ]]; then
    local pkg_name
    pkg_name="$(get_ros2_package_name "${pkg_dir}")"

    echo "[build] Cleaning ROS2 package: ${pkg_name}"

    # Find build directory for this package
    local build_dir
    build_dir="$(find "${ROS2_BUILD_ROOT}" -type d -name "${pkg_name}" -maxdepth 2 2>/dev/null | head -n 1 || true)"

    local cleaned=false

    # Use CMake's uninstall target if available (most reliable method)
    if [[ -n "${build_dir}" && -d "${build_dir}" ]]; then
      echo "[build] Using CMake uninstall target for: ${pkg_name}"
      if cmake --build "${build_dir}" --target uninstall >/dev/null 2>&1; then
        echo "[build] Uninstalled files using CMake uninstall target"
        cleaned=true
      fi

      # Clean build files using CMake clean target
      echo "[build] Cleaning build files using CMake clean target"
      if cmake --build "${build_dir}" --target clean >/dev/null 2>&1; then
        echo "[build] Build files cleaned successfully"
        cleaned=true
      else
        echo "[build] CMake clean target failed; falling back to manual cleanup"
      fi
    fi

    # Always remove build/install outputs that CMake clean/uninstall may leave behind.
    local build_dirs
    build_dirs="$(find "${ROS2_BUILD_ROOT}" -type d -name "${pkg_name}" 2>/dev/null || true)"
    if [[ -n "${build_dirs}" ]]; then
      echo "[build] Removing build directories..."
      while IFS= read -r dir; do
        [[ -n "${dir}" ]] && rm -rf "${dir}"
      done <<< "${build_dirs}"
      cleaned=true
    fi

    if [[ -d "${PREFIX}/share/${pkg_name}" ]]; then
      echo "[build] Removing install directory: ${PREFIX}/share/${pkg_name}"
      rm -rf "${PREFIX}/share/${pkg_name}"
      cleaned=true
    fi

    local lib_files
    lib_files="$(find "${PREFIX}/lib" -name "lib${pkg_name}.so*" 2>/dev/null || true)"
    if [[ -n "${lib_files}" ]]; then
      echo "[build] Removing library files..."
      while IFS= read -r file; do
        [[ -n "${file}" ]] && rm -f "${file}"
      done <<< "${lib_files}"
      cleaned=true
    fi

    if [[ -d "${PREFIX}/lib/${pkg_name}" ]]; then
      echo "[build] Removing library directory: ${PREFIX}/lib/${pkg_name}"
      rm -rf "${PREFIX:?}/lib/${pkg_name}"
      cleaned=true
    fi

    local ament_files
    ament_files="$(find "${PREFIX}/share/ament_index" -name "${pkg_name}" -type f 2>/dev/null || true)"
    if [[ -n "${ament_files}" ]]; then
      echo "[build] Removing ament_index entries..."
      while IFS= read -r file; do
        [[ -n "${file}" ]] && rm -f "${file}"
      done <<< "${ament_files}"
      cleaned=true
    fi

    if [[ -f "${PREFIX}/share/colcon-core/packages/${pkg_name}" ]]; then
      echo "[build] Removing colcon package registry entry..."
      rm -f "${PREFIX}/share/colcon-core/packages/${pkg_name}"
      cleaned=true
    fi

    if [[ "${cleaned}" == false ]]; then
      echo "[build] No build or install files found for package: ${pkg_name}"
    else
      echo "[build] Clean complete"
    fi

    return 0
    fi
  fi

  # CMake package cleanup (package.xml with build_type=cmake, or plain CMakeLists.txt)
  if [[ -f "${pkg_dir}/CMakeLists.txt" ]]; then
    local pkg_key="${pkg_dir#"${REPO_ROOT}"/}"
    local build_dir
    build_dir="$(pkg_build_dir "${pkg_key}")"
    echo "[build] Cleaning CMake package build dir: ${build_dir}"
          rm -rf "${build_dir}"
        echo "[build] Clean complete"
    return 0
  fi

  echo "[build] ERROR: No valid package identifier found" >&2
  exit 1
}

# ============================================================================
# Clean functions
# ============================================================================

clean_build() {
  local clean_type="${1:-all}"

  case "${clean_type}" in
    cmake|C)
      echo "[build] Cleaning CMake build directories"
      rm -rf "${CM_BUILD_ROOT}" "${OUTPUT_ROOT}"/build-*
      ;;
    ros2|R)
      echo "[build] Cleaning ROS2 build directories and logs"
      rm -rf "${ROS2_BUILD_ROOT}" "${ROS2_LOG_ROOT}"
      rm -rf "${OUTPUT_ROOT}/ros2_build" "${OUTPUT_ROOT}/ros2_log"
      ;;
    all|*)
      echo "[build] Cleaning all build directories"
      rm -rf "${OUTPUT_ROOT}/build" "${OUTPUT_ROOT}"/build-*
      rm -rf "${ROS2_LOG_ROOT}" "${OUTPUT_ROOT}/ros2_build" "${OUTPUT_ROOT}/ros2_log"
      # Clean default prefixes (staging + rootfs) when they are under OUTPUT_ROOT
      if [[ "${STAGING_PREFIX}" == "${OUTPUT_ROOT}/staging" ]]; then
        echo "[build] Cleaning staging prefix: ${STAGING_PREFIX}"
        rm -rf "${STAGING_PREFIX}"
      fi
      if [[ "${ROOTFS_PREFIX}" == "${OUTPUT_ROOT}/rootfs" ]]; then
        echo "[build] Cleaning rootfs prefix: ${ROOTFS_PREFIX}"
        rm -rf "${ROOTFS_PREFIX}"
      fi
      ;;
  esac
}

# ============================================================================
# Main function
# ============================================================================

main() {
  local original_args=("$@")

  # Parse global flags that may appear before the command.
  # Supported:
  #   -jN
  #   -v
  #   --log=quiet|normal|verbose
  #   --py, -py
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -j)
        echo "[build] ERROR: -j requires a positive integer, e.g. -j8" >&2
        exit 1
        ;;
      -j*)
        PARALLEL_JOBS="${1#-j}"
        if [[ ! "${PARALLEL_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
          echo "[build] ERROR: invalid -j value: ${PARALLEL_JOBS}. Use a positive integer, e.g. -j8" >&2
          exit 1
        fi
        export PARALLEL_JOBS
        export SROBOTIS_PARALLEL_JOBS_EXPLICIT=1
        shift
        ;;
      -v)
        LOG_LEVEL="verbose"
        export LOG_LEVEL
        shift
        ;;
      --log=*)
        LOG_LEVEL="${1#--log=}"
        export LOG_LEVEL
        shift
        ;;
      --py|-py)
        _want_python_wheels=1
        shift
        ;;
      --)
        shift
        break
        ;;
      -*)
        echo "[build] ERROR: Unknown global option: $1" >&2
        echo "Use 'build.sh help' for usage" >&2
        exit 1
        ;;
      *)
        break
        ;;
    esac
  done

  local cmd="${1:-help}"
  shift || true

  # Load build configuration if BUILD_TARGET is set
  load_build_config

  if docker_build_command_needs_wrapper "${cmd}" "$@" && docker_build_host_enabled; then
    if [[ "${SROBOTIS_PARALLEL_JOBS_EXPLICIT:-0}" != "1" && "${SROBOTIS_DOCKER_MAX_JOBS:-1}" == "1" ]]; then
      PARALLEL_JOBS="$(nproc)"
      export PARALLEL_JOBS
      export SROBOTIS_PARALLEL_JOBS_EXPLICIT=1
    fi
    run_build_in_docker "${original_args[@]}"
    exit $?
  fi

  # Whether this invocation builds non-ROS2 packages without a target config.
  # Used by build/common.sh to decide whether no-target dependency discovery
  # should scan all buildable non-ROS2 packages.
  case "${cmd}" in
    all|cmake|C)
      export BUILD_NEEDS_NONROS2=1
      ;;
    *)
      export BUILD_NEEDS_NONROS2=0
      ;;
  esac

  # Whether this invocation needs ROS2 system dependencies.
  # Used by build/common.sh to conditionally include build/package_ros2.xml.
  #
  # Rules:
  # - ros2/R always needs ROS2 deps
  # - all without a selected target builds full ROS2 workspaces when they exist
  # - all with a selected target needs ROS2 deps only if that target enables ROS2 packages
  # - cmake/C never needs ROS2 deps
  case "${cmd}" in
    ros2|R)
      export BUILD_NEEDS_ROS2=1
      ;;
    all)
      if [[ -z "${BUILD_CONFIG_FILE:-}" || ! -f "${BUILD_CONFIG_FILE}" ]]; then
        if [[ -d "${REPO_ROOT}/middleware/ros2" || -d "${REPO_ROOT}/application/ros2" ]]; then
          export BUILD_NEEDS_ROS2=1
        else
          export BUILD_NEEDS_ROS2=0
        fi
      elif target_needs_ros2; then
        export BUILD_NEEDS_ROS2=1
      else
        export BUILD_NEEDS_ROS2=0
      fi
      ;;
    *)
      export BUILD_NEEDS_ROS2=0
      ;;
  esac

  # Check and install system dependencies (skip for clean/help).
  # For "package" we only check that package's deps later in the package branch.
  if [[ "${cmd}" != "clean" && "${cmd}" != "help" && "${cmd}" != "--help" && "${cmd}" != "-h" && "${cmd}" != "package" && "${cmd}" != "pkg" ]]; then
    if ! check_and_install_dependencies; then
      echo "[build] ERROR: System dependency check/installation failed" >&2
      echo "[build] Please install missing dependencies manually or check configuration" >&2
      exit 1
    fi
    if [[ "${SROBOTIS_IN_DOCKER_BUILD:-0}" == "1" && "${SROBOTIS_DEPS_ONLY:-0}" == "1" ]]; then
      echo "[deps] Dependency-only check complete"
      return 0
    fi
  fi

  case "${cmd}" in
    all)
      build_nonros2_enabled_packages "${_want_python_wheels:-0}"
      build_ros2_middleware
      build_ros2_applications
      # After a full build, automatically generate the runtime rootfs prefix.
      deploy_rootfs
      ;;

    cmake|C)
      build_nonros2_enabled_packages "${_want_python_wheels:-0}"
      ;;

    ros2|R)
      build_ros2_middleware
      build_ros2_applications
      ;;

    package|pkg)
      local pkg_dir="${1:-}"
      local action="${2:-build}"
      local extra_arg="${3:-}"
      local deps_mode="none"

      [[ -z "${pkg_dir}" ]] && { echo "[build] ERROR: Package directory required" >&2; exit 1; }
      if [[ -n "${extra_arg}" ]]; then
        echo "[build] ERROR: Unexpected package argument: ${extra_arg}" >&2
        echo "Use 'build.sh help' for usage" >&2
        exit 1
      fi

      case "${action}" in
        --with-deps)
          deps_mode="with"
          action="build"
          ;;
        --deps)
          deps_mode="only"
          action="build"
          ;;
        clean|build)
          ;;
        "")
          action="build"
          ;;
        *)
          echo "[build] ERROR: Unknown package action: ${action}" >&2
          echo "Use 'build.sh help' for usage" >&2
          exit 1
          ;;
      esac

      if [[ "${action}" == "clean" ]]; then
        if [[ "${deps_mode}" != "none" ]]; then
          echo "[build] ERROR: package clean cannot be combined with dependency build modes" >&2
          exit 1
        fi
        clean_single_package "${pkg_dir}"
      else
        if ! check_and_install_dependencies_for_package "${pkg_dir}" "${deps_mode}"; then
          echo "[build] ERROR: System dependency check/installation failed for this package" >&2
          exit 1
        fi
        if [[ "${SROBOTIS_IN_DOCKER_BUILD:-0}" == "1" && "${SROBOTIS_DEPS_ONLY:-0}" == "1" ]]; then
          echo "[deps] Dependency-only check complete"
          return 0
        fi
        local is_ros2_package=0
        local pkg_build_type=""
        if [[ -d "${pkg_dir}" && -f "${pkg_dir}/package.xml" ]]; then
          pkg_build_type="$(get_package_build_type "${pkg_dir}" || true)"
          if [[ "${pkg_build_type}" == "ament_cmake" || "${pkg_build_type}" == "ament_python" ]]; then
            is_ros2_package=1
          fi
        fi
        if [[ "${deps_mode}" != "none" ]]; then
          local abs_pkg_dir
          [[ ! -d "${pkg_dir}" ]] && { echo "[build] ERROR: Package directory not found: ${pkg_dir}" >&2; exit 1; }
          abs_pkg_dir="$(cd "${pkg_dir}" && pwd)"
          if [[ "${abs_pkg_dir}" != "${REPO_ROOT}/"* ]]; then
            echo "[build] ERROR: Package directory must be inside repo: ${abs_pkg_dir}" >&2
            exit 1
          fi
          local pkg_key="${abs_pkg_dir#"${REPO_ROOT}"/}"
          if [[ "${is_ros2_package}" == "1" ]]; then
            local sdk_dep
            while IFS= read -r sdk_dep; do
              [[ -n "${sdk_dep}" ]] || continue
              (
                if [[ "${SROBOTIS_CROSS_BUILD:-0}" != "1" ]]; then
                  unset SROBOTIS_CMAKE_EXTRA_ARGS
                fi
                build_nonros2_package_deps "${sdk_dep}" 1 "${_want_python_wheels:-0}"
              )
            done < <(read_ros2_sdk_nonros2_deps "${pkg_key}")
            if [[ "${deps_mode}" == "with" ]]; then
              SROBOTIS_ROS2_PACKAGE_DEPS_MODE=with build_single_package "${pkg_dir}"
            else
              SROBOTIS_ROS2_PACKAGE_DEPS_MODE=only build_single_package "${pkg_dir}"
            fi
          else
            (
              if [[ "${SROBOTIS_CROSS_BUILD:-0}" != "1" ]]; then
                unset SROBOTIS_CMAKE_EXTRA_ARGS
              fi
              build_nonros2_package_deps "${pkg_key}" 0 "${_want_python_wheels:-0}"
            )
          fi
        fi
        if [[ "${deps_mode}" != "only" && ( "${deps_mode}" != "with" || "${is_ros2_package}" != "1" ) ]]; then
          build_single_package "${pkg_dir}"
        fi
      fi
      ;;

    clean)
      clean_build "${1:-all}"
      ;;

    deploy-rootfs|rootfs)
      deploy_rootfs
      ;;

    help|--help|-h)
      cat <<EOF
Usage: build.sh <command> [args...]

Commands:
  all                   Build all (CMake + ROS2)
  cmake, C              Build CMake packages only
  ros2, R               Build ROS2 packages only
  package <dir> [clean|--with-deps|--deps]
                        Build or clean single package; --with-deps builds package deps first,
                        --deps builds only package deps
  deploy-rootfs, rootfs Generate ${OUTPUT_ROOT}/rootfs from ${OUTPUT_ROOT}/staging (keeps \$(ros2 run) working)
  clean [all|cmake|ros2] Clean build directories

Options:
  -jN                  Parallel build jobs (e.g. -j8)
  -v                   Verbose: print full CMake/colcon output to console
  --log=LEVEL          Logging level: quiet|normal|verbose (default: quiet)
  --py, -py            After each non-ROS2 install, build Python wheels where applicable

Environment variables:
  STAGING_PREFIX      Full install prefix for development/build (default: ${OUTPUT_ROOT}/staging)
  ROOTFS_PREFIX       Runtime deploy prefix (default: ${OUTPUT_ROOT}/rootfs)
  PREFIX              Alias for STAGING_PREFIX (default: ${OUTPUT_ROOT}/staging)
  PARALLEL_JOBS       Parallel build jobs (default: $(nproc))
  LOG_LEVEL           quiet|normal|verbose (default: quiet)
  LOG_ROOT            Log directory (default: ${OUTPUT_ROOT}/log)
  LOG_TEE             1 to tee output into logs when verbose (default: 1)
  LOG_TAIL_LINES      Tail lines printed from logs on error (default: 200)
  LOG_SHOW_ENTER      1 to print per-package "Start: ..." lines (default: 1)
  LOG_SHOW_DONE       1 to print per-package "End ..." lines (default: 1)
  ROS_DISTRO          ROS2 distribution (default: ${ROS_DISTRO})
  ROS_SETUP           ROS setup script (default: ${ROS_SETUP})
  AUTO_INSTALL_DEPS   Auto-install missing dependencies (yes/true to enable)
  DEBUG_DEPS          Show detailed dependency check output (1 to enable)
  SROBOTIS_USE_DOCKER_BUILD
                      1/yes/true/on to run selected k1/k3 builds in Bianbu Docker;
                      source build/envsetup.sh resets it to 0, use m_enable_docker_build to enable
  SROBOTIS_DOCKER_RUN_AS_ROOT
                      1 to run the build step inside Docker as root; default uses host uid/gid for owned outputs
  SROBOTIS_DOCKER_PLATFORM
                      Docker platform for Bianbu images (default: linux/riscv64)
  SROBOTIS_DOCKER_CONTAINER_NAME
                      Existing/created Docker container name; default includes current SDK path key
  SROBOTIS_DOCKER_MOUNT_SRC
                      Host path mounted into Docker (default: current SDK root)
  SROBOTIS_DOCKER_MOUNT_DST
                      Container mount path (default: same as SROBOTIS_DOCKER_MOUNT_SRC)
  SROBOTIS_DOCKER_FIX_OUTPUT_OWNER
                      1 to chown OUTPUT_ROOT back to host uid/gid after root dependency install (default: 1)
  SROBOTIS_DOCKER_MAX_JOBS
                      1 to use host nproc for Docker builds when -j is not explicit (default: 1)

Examples:
  PARALLEL_JOBS=8 ./build/build.sh all
  ./build/build.sh package components/gui
  ./build/build.sh clean cmake
  AUTO_INSTALL_DEPS=yes ./build/build.sh all
  ./build/build.sh --py cmake
EOF
      ;;

    *)
      echo "[build] ERROR: Unknown command: ${cmd}" >&2
      echo "Use 'build.sh help' for usage" >&2
      exit 1
      ;;
  esac
}

main "$@"
