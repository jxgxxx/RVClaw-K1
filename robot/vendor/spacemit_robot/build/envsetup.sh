#!/usr/bin/env bash
#
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#

# This script is meant to be *sourced*, not executed:
#   source build/envsetup.sh
#
# It sets common environment variables and defines convenient
# helper commands for working with the S-Robot SDK.

# For zsh compatibility: make arrays 0-indexed like bash
if [[ -n "${ZSH_VERSION:-}" ]]; then
  setopt KSH_ARRAYS
fi

# Detect repo root (one level above this script)
# Compatible with bash, zsh, and other POSIX shells
_detect_script_dir() {
  local script_path=""
  # Multi-shell script path detection (bash/zsh/ksh). Disable checks for non-bash syntax.
  # shellcheck disable=SC2296,SC1072,SC1073,SC1009
  if [[ -n "${BASH_SOURCE[0]:-}" ]]; then
    script_path="${BASH_SOURCE[0]}"
  elif [[ -n "${ZSH_VERSION:-}" ]]; then
    script_path="${(%):-%x}"
  elif [[ -n "${.sh.file:-}" ]]; then
    script_path="${.sh.file}"
  elif [[ -n "$0" && "$0" != "-bash" && "$0" != "-zsh" && "$0" != "bash" && "$0" != "zsh" ]]; then
    script_path="$0"
  fi

  # Validate we got a path
  if [[ -z "${script_path}" ]]; then
    echo "[env] ERROR: Cannot detect script location. Please source this script from bash or zsh." >&2
    return 1
  fi

  # Resolve to absolute path and go one level up
  cd "$(dirname "${script_path}")/.." && pwd
}

SROBOTIS_ROOT="$(_detect_script_dir)"
export SROBOTIS_ROOT
if [[ $? -ne 0 || -z "${SROBOTIS_ROOT}" ]]; then
  echo "[env] ERROR: Failed to initialize SROBOTIS_ROOT" >&2
  return 1
fi

# Layout:
#   ${SROBOTIS_ROOT}/output/
#     staging/ - full install prefix for development/build (REQUIRED)
#     rootfs/  - runtime deploy prefix (generated from staging; still supports `ros2 run`)
#     build/  - CMake build tree
#     build-* - per-component build trees
export SROBOTIS_OUTPUT_ROOT="${SROBOTIS_ROOT}/output"
export SROBOTIS_OUTPUT_STAGING="${SROBOTIS_OUTPUT_ROOT}/staging"
export SROBOTIS_OUTPUT_ROOTFS="${SROBOTIS_OUTPUT_ROOT}/rootfs"
export SROBOTIS_OUTPUT="${SROBOTIS_OUTPUT_STAGING}"

# Docker build is opt-in for each freshly sourced SDK shell. Re-sourcing this
# file intentionally disables it until m_enable_docker_build is run again.
export SROBOTIS_USE_DOCKER_BUILD=0

# Cross build is opt-in for each freshly sourced SDK shell. Re-sourcing this
# file intentionally disables it until m_enable_cross_build is run again.
export SROBOTIS_USE_CROSS_BUILD=0

if [[ -n "${PREFIX-}" ]]; then
  case "${PREFIX}" in
    "${SROBOTIS_OUTPUT_STAGING}"|"${SROBOTIS_OUTPUT_ROOTFS}"|"${SROBOTIS_OUTPUT_ROOT}"/*)
      # looks like a valid prefix for this repo; keep as-is
      ;;
    *)
      echo "[env] Detected PREFIX from another environment: ${PREFIX}"
      echo "[env] Resetting PREFIX to ${SROBOTIS_OUTPUT} for repo ${SROBOTIS_ROOT}"
      unset PREFIX
      ;;
  esac
fi
export PREFIX="${PREFIX:-${SROBOTIS_OUTPUT}}"

# Default ROS2 distribution for this repo (can be overridden before sourcing)
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"

# Make sure ~/.local/bin (where colcon is often installed) is on PATH
if [[ -d "${HOME}/.local/bin" ]]; then
  case ":${PATH}:" in
    *":${HOME}/.local/bin:"*) ;;
    *) export PATH="${HOME}/.local/bin:${PATH}" ;;
  esac
fi

# Ensure the SDK bin directory is on PATH so that executables
# (e.g. vlm_smolvlm) can be found and executed directly.
case ":${PATH}:" in
  *":${PREFIX}/bin:"*) ;;
  *)
    export PATH="${PREFIX}/bin:${PATH}"
    ;;
esac

# Ensure the SDK lib directory is on LD_LIBRARY_PATH so that dynamic
# libraries (e.g. libMlinkDevice.so) can be found at runtime.
case ":${LD_LIBRARY_PATH-}:" in
  *":${PREFIX}/lib:"*) ;;
  *)
    if [[ -n "${LD_LIBRARY_PATH-}" ]]; then
      export LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH}"
    else
      export LD_LIBRARY_PATH="${PREFIX}/lib"
    fi
    ;;
esac

# Ensure the SDK python directory is on PYTHONPATH so that pure-Python
# modules (e.g. mlink_pydevice) can be imported.
if [[ -d "${PREFIX}/python" ]]; then
  case ":${PYTHONPATH-}:" in
    *":${PREFIX}/python:"*) ;;
    *)
      if [[ -n "${PYTHONPATH-}" ]]; then
        export PYTHONPATH="${PREFIX}/python:${PYTHONPATH}"
      else
        export PYTHONPATH="${PREFIX}/python"
      fi
      ;;
  esac
fi

# Convenience function: build a Python virtualenv for a given application
# directory by reading its pyproject.toml and:
#   - installing local SDKs (pipecat-ai, mlink-gateway, pipecat-ext, etc.)
#     from source in editable mode when they exist in this repo;
#   - installing all other dependencies from Python package indexes.
# Usage (after sourcing this file):
#   m_env_build xxxx
m_env_build() {
  if [[ $# -lt 1 ]]; then
    echo "Usage: m_env_build <app_dir>" >&2
    echo "  e.g. m_env_build xxxx" >&2
    return 1
  fi
  (
    cd "${SROBOTIS_ROOT}" || exit 1
    ./build/python_env_build.sh "$@"
  )
}

# Convenience function: enable or disable Docker builds in the current shell.
# Usage (after sourcing this file):
#   m_enable_docker_build
#   m_enable_docker_build disable
m_enable_docker_build() {
  local action="${1:-enable}"

  if [[ $# -gt 1 ]]; then
    echo "Usage: m_enable_docker_build [disable]" >&2
    return 1
  fi

  case "${action}" in
    enable|on|1|true|yes)
      export SROBOTIS_USE_DOCKER_BUILD=1
      echo "[docker] Docker build enabled for this shell."
      ;;
    disable|off|0|false|no)
      export SROBOTIS_USE_DOCKER_BUILD=0
      echo "[docker] Docker build disabled for this shell."
      ;;
    -h|--help)
      cat <<EOF
Usage: m_enable_docker_build [disable]

Enable Bianbu Docker builds for this sourced SDK shell.
Re-source build/envsetup.sh to reset Docker builds back to disabled.
EOF
      ;;
    *)
      echo "[docker] ERROR: Unknown option: ${action}" >&2
      echo "Usage: m_enable_docker_build [disable]" >&2
      return 1
      ;;
  esac
}

# Convenience function: enable or disable cross builds in the current shell.
# Usage (after sourcing this file):
#   m_enable_cross_build
#   m_enable_cross_build disable
m_enable_cross_build() {
  local action="${1:-enable}"

  if [[ $# -gt 1 ]]; then
    echo "Usage: m_enable_cross_build [disable]" >&2
    return 1
  fi

  case "${action}" in
    enable|on|1|true|yes)
      export SROBOTIS_USE_CROSS_BUILD=1
      echo "[cross] Cross build enabled for this shell."
      ;;
    disable|off|0|false|no)
      export SROBOTIS_USE_CROSS_BUILD=0
      echo "[cross] Cross build disabled for this shell."
      ;;
    -h|--help)
      cat <<EOF
Usage: m_enable_cross_build [disable]

Enable cross builds for this sourced SDK shell. m/mm route through
build/cross_build.sh and require a k1/k3 target selected with lunch.
EOF
      ;;
    *)
      echo "[cross] ERROR: Unknown option: ${action}" >&2
      echo "Usage: m_enable_cross_build [disable]" >&2
      return 1
      ;;
  esac
}

# Convenience function: create a wheel-build Python virtual environment
# for SDK subprojects using uv.
# Usage (after sourcing this file):
#   venv <python_version>
# Example:
#   venv 3.13.12
venv() {
  if [[ $# -ne 1 ]]; then
    echo "Usage: venv <python_version>" >&2
    echo "  e.g. venv 3.13.12" >&2
    return 1
  fi

  local python_version="$1"
  local venv_dir="${SROBOTIS_ROOT}/.venv"
  local uv_bin=""

  if command -v uv >/dev/null 2>&1; then
    uv_bin="$(command -v uv)"
  else
    echo "[venv] uv not found, installing uv ..."
    if ! command -v curl >/dev/null 2>&1; then
      echo "[venv] ERROR: curl is required to install uv." >&2
      return 1
    fi

    if ! curl -LsSf https://astral.sh/uv/install.sh | sh; then
      echo "[venv] ERROR: failed to install uv." >&2
      return 1
    fi

    if [[ -x "${HOME}/.local/bin/uv" ]]; then
      uv_bin="${HOME}/.local/bin/uv"
    elif command -v uv >/dev/null 2>&1; then
      uv_bin="$(command -v uv)"
    else
      echo "[venv] ERROR: uv was installed but is still not available in PATH." >&2
      return 1
    fi
  fi

  export UV_EXTRA_INDEX_URL="https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple"
  export UV_INDEX_URL="https://mirrors.aliyun.com/pypi/simple"

  if [[ -d "${venv_dir}" ]]; then
    echo "[venv] Removing existing virtual environment: ${venv_dir}"
    rm -rf "${venv_dir}"
  fi

  (
    cd "${SROBOTIS_ROOT}" || exit 1
    "${uv_bin}" venv --python "${python_version}" "${venv_dir}"
  ) || {
    echo "[venv] ERROR: failed to create virtual environment with Python ${python_version}." >&2
    return 1
  }

  # shellcheck disable=SC1091
  source "${venv_dir}/bin/activate" || {
    echo "[venv] ERROR: failed to activate ${venv_dir}." >&2
    return 1
  }

  if ! "${uv_bin}" pip install \
    build \
    packaging \
    pip \
    pybind11 \
    pyproject_hooks \
    setuptools \
    wheel; then
    echo "[venv] ERROR: failed to install wheel build dependencies." >&2
    return 1
  fi

  echo "[venv] Ready: ${venv_dir}"
  echo "[venv] Python version: ${python_version}"
  echo "[venv] UV_INDEX_URL=${UV_INDEX_URL}"
  echo "[venv] UV_EXTRA_INDEX_URL=${UV_EXTRA_INDEX_URL}"
  echo "[venv] Virtual environment activated."
  echo "[venv] You can now cd into vad/asr/tts/vision etc. and run 'mm'."
}

# Convenience function: change directory to repo root
#   croot
croot() {
  cd "${SROBOTIS_ROOT}" || return
}

# Convenience function: change directory to components/
#   ccomponent
ccomponent() {
  cd "${SROBOTIS_ROOT}/components" || return
}

# Convenience function: change directory to application/
#   capp
capp() {
  cd "${SROBOTIS_ROOT}/application" || return
}

# Convenience function: fully prepare ROS2 + SDK + overlay environment
# in the *current* shell.
# Usage (after sourcing this file):
#   sros2_setup
# Then you can run:
#   ros2 run srobot_mlink_demo mlink_demo_node
sros2_setup() {
  # 1) Source system ROS2 setup (e.g. /opt/ros/humble/setup.bash)
  if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[env] ROS setup script does not exist: ${ROS_SETUP}" >&2
    echo "[env] Please install ROS2 or set ROS_SETUP to a valid setup.bash." >&2
    return 1
  fi

  # Respect existing 'set -u' in the current shell (if any) while sourcing ROS_SETUP
  # Source from the script's directory to ensure relative paths work correctly
  local ros_setup_dir
  ros_setup_dir="$(dirname "${ROS_SETUP}")"
  local saved_pwd="${PWD}"
  cd "${ros_setup_dir}" || return 1

  case "$-" in
    *u*)
      set +u
      # shellcheck source=/dev/null
      source "${ROS_SETUP}"
      set -u
      ;;
    *)
      # shellcheck source=/dev/null
      source "${ROS_SETUP}"
      ;;
  esac

  cd "${saved_pwd}" || return 1

  # 2) Source SDK-installed ROS2 overlay under PREFIX (if available)
  local sdk_ros_setup=""
  if [[ -f "${PREFIX}/local_setup.bash" ]]; then
    sdk_ros_setup="${PREFIX}/local_setup.bash"
  elif [[ -f "${PREFIX}/setup.bash" ]]; then
    sdk_ros_setup="${PREFIX}/setup.bash"
  fi

  if [[ -z "${sdk_ros_setup}" ]]; then
    echo "[env] No ROS2 local_setup.bash or setup.bash found under PREFIX=${PREFIX}" >&2
    echo "[env] Did you run 'm -R' or './build/build.sh ros2'?" >&2
    return 1
  fi

  # Source from the script's directory to ensure relative paths work correctly
  local sdk_setup_dir
  sdk_setup_dir="$(dirname "${sdk_ros_setup}")"
  saved_pwd="${PWD}"
  cd "${sdk_setup_dir}" || return 1

  case "$-" in
    *u*)
      set +u
      # shellcheck source=/dev/null
      source "${sdk_ros_setup}"
      set -u
      ;;
    *)
      # shellcheck source=/dev/null
      source "${sdk_ros_setup}"
      ;;
  esac

  cd "${saved_pwd}" || return 1

  echo "[env] ROS2 environment ready:"
  echo "      ROS_SETUP=${ROS_SETUP}"
  echo "      SDK ROS2 setup=${sdk_ros_setup}"
}

# Interactive menu to select build target configuration
# Usage: lunch [target_name]
#   If target_name is provided, select it directly
#   Otherwise, show interactive menu
lunch() {
  # Handle help
  if [[ $# -eq 1 && ("$1" == "-h" || "$1" == "--help") ]]; then
    cat <<EOF
Usage: lunch [target_name]

Select build target configuration:
  lunch                    Show interactive menu to select configuration
  lunch <target>           Select target directly (e.g., lunch k3-com260-minimal)

The selected target is stored in BUILD_TARGET environment variable.
Configuration files are located in: ${SROBOTIS_ROOT}/target
EOF
    return 0
  fi

  local target_dir="${SROBOTIS_ROOT}/target"

  if [[ ! -d "${target_dir}" ]]; then
    echo "[lunch] ERROR: target directory not found: ${target_dir}" >&2
    return 1
  fi

  # If target is provided as argument, use it directly
  if [[ $# -gt 0 ]]; then
    local target_name="$1"
    local target_file="${target_dir}/${target_name}"

    # Add .json extension if not present
    if [[ "${target_name}" != *.json ]]; then
      target_file="${target_dir}/${target_name}.json"
    fi

    if [[ ! -f "${target_file}" ]]; then
      echo "[lunch] ERROR: Configuration file not found: ${target_file}" >&2
      return 1
    fi

    export BUILD_TARGET="${target_name%.json}"
    export BUILD_TARGET_FILE="${target_file}"
    echo "[lunch] Selected: ${BUILD_TARGET}"

    # Parse and display config info
    if command -v jq >/dev/null 2>&1; then
      local board product desc
      board="$(jq -r '.board // "unknown"' "${target_file}" 2>/dev/null)"
      product="$(jq -r '.product // "unknown"' "${target_file}" 2>/dev/null)"
      desc="$(jq -r '.description // ""' "${target_file}" 2>/dev/null)"
      echo "[lunch] Board: ${board}, Product: ${product}"
      [[ -n "${desc}" ]] && echo "[lunch] ${desc}"
    fi
    return 0
  fi

  # Interactive menu
  echo ""
  echo "You're building on $(uname -s)"
  echo ""
  echo "Lunch menu... pick a combo:"
  echo ""

  # Find all JSON files in target directory
  local configs=()
  local config_files=()
  local i=1

  while IFS= read -r -d '' file; do
    local basename="${file##*/}"
    local name="${basename%.json}"
    configs+=("${name}")
    config_files+=("${file}")

    # Try to extract board and product for display
    local display_name="${name}"
    if command -v jq >/dev/null 2>&1; then
      local board product
      board="$(jq -r '.board // ""' "${file}" 2>/dev/null)"
      product="$(jq -r '.product // ""' "${file}" 2>/dev/null)"
      if [[ -n "${board}" && -n "${product}" ]]; then
        display_name="${board}-${product}"
      fi
    fi

    printf "     %d\t%s\n" "${i}" "${display_name}"
    ((i++))
  done < <(find "${target_dir}" -maxdepth 1 -type f -name "*.json" -print0 | sort -z)

  if [[ ${#configs[@]} -eq 0 ]]; then
    echo "[lunch] ERROR: No configuration files found in ${target_dir}" >&2
    return 1
  fi

  # Get default (first config or previously selected)
  local default_target="${BUILD_TARGET:-${configs[0]}}"
  local default_num=1
  for ((i=0; i<${#configs[@]}; i++)); do
    if [[ "${configs[$i]}" == "${default_target}" ]]; then
      default_num=$((i + 1))
      break
    fi
  done

  echo ""
  # Use printf instead of read -p for bash/zsh compatibility
  printf 'Which would you like? [Default %s]: ' "${default_target}"
  read -r selection

  # Handle empty input (use default)
  if [[ -z "${selection}" ]]; then
    selection="${default_num}"
  fi

  # Handle numeric selection
  if [[ "${selection}" =~ ^[0-9]+$ ]]; then
    local idx=$((selection - 1))
    if [[ ${idx} -ge 0 && ${idx} -lt ${#configs[@]} ]]; then
      export BUILD_TARGET="${configs[$idx]}"
      export BUILD_TARGET_FILE="${config_files[$idx]}"
    else
      echo "[lunch] ERROR: Invalid selection: ${selection}" >&2
      return 1
    fi
  else
    # Handle name-based selection
    local found=false
    for ((i=0; i<${#configs[@]}; i++)); do
      if [[ "${configs[$i]}" == "${selection}" ]] || \
         [[ "${configs[$i]}" == "${selection}.json" ]] || \
         [[ "${configs[$i]%.json}" == "${selection}" ]]; then
        export BUILD_TARGET="${configs[$i]}"
        export BUILD_TARGET_FILE="${config_files[$i]}"
        found=true
        break
      fi
    done

    if [[ "${found}" != true ]]; then
      echo "[lunch] ERROR: Configuration not found: ${selection}" >&2
      return 1
    fi
  fi

  echo ""
  echo "[lunch] Selected: ${BUILD_TARGET}"

  # Parse and display config info
  if command -v jq >/dev/null 2>&1; then
    local board product desc
    board="$(jq -r '.board // "unknown"' "${BUILD_TARGET_FILE}" 2>/dev/null)"
    product="$(jq -r '.product // "unknown"' "${BUILD_TARGET_FILE}" 2>/dev/null)"
    desc="$(jq -r '.description // ""' "${BUILD_TARGET_FILE}" 2>/dev/null)"
    echo "[lunch] Board: ${board}, Product: ${product}"
    [[ -n "${desc}" ]] && echo "[lunch] ${desc}"
  fi
  echo ""
}

# Helper function to run build.sh with environment variables
_run_build() {
  local build_script="./build/build.sh"
  if [[ "${SROBOTIS_USE_CROSS_BUILD:-0}" == "1" ]]; then
    build_script="./build/cross_build.sh"
  fi
  PREFIX="${PREFIX}" ROS_DISTRO="${ROS_DISTRO}" ROS_SETUP="${ROS_SETUP}" \
    PARALLEL_JOBS="${PARALLEL_JOBS:-$(nproc)}" \
    BUILD_TARGET="${BUILD_TARGET:-}" BUILD_TARGET_FILE="${BUILD_TARGET_FILE:-}" \
    "${build_script}" "$@"
}

# Build command for repo root
# Supports full build, filtered build, and clean
m() {
  local current_dir
  current_dir="$(pwd)"

  if [[ "$current_dir" != "$SROBOTIS_ROOT" ]] || [[ ! -f "${SROBOTIS_ROOT}/build/build.sh" ]]; then
    echo "[m] ERROR: Must be run from repo root" >&2
    return 1
  fi

  local build_type="all"
  local clean_mode=false
  local parallel_jobs="${PARALLEL_JOBS:-$(nproc)}"
  local log_level_arg=""
  local py_flag=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -C)
        build_type="cmake"
        shift
        ;;
      -R)
        build_type="ros2"
        shift
        ;;
      -j*)
        parallel_jobs="${1#-j}"
        shift
        ;;
      -v)
        log_level_arg="--log=verbose"
        shift
        ;;
      --log=*)
        log_level_arg="$1"
        shift
        ;;
      clean)
        clean_mode=true
        shift
        ;;
      -py)
        py_flag=(--py)
        shift
        ;;
      -h|--help)
        cat <<EOF
Usage: m [options] [clean]

Build commands (run from repo root):
  m                   Build all (CMake + ROS2)
  m -C                Build CMake packages only
  m -R                Build ROS2 packages only
  m -py               Opt-in: build Python wheels after non-ROS2 installs
  m -j8               Use 8 parallel jobs
  m -v                Verbose: print full CMake/colcon output to console
  m --log=LEVEL       Set logging level: quiet|normal|verbose (default: quiet)
  m clean             Clean all
  m -C clean          Clean CMake only
  m -R clean          Clean ROS2 only
EOF
        return 0
        ;;
      *)
        echo "[m] ERROR: Unknown option: $1" >&2
        return 1
        ;;
    esac
  done

  export PARALLEL_JOBS="${parallel_jobs}"
  (
    cd "${SROBOTIS_ROOT}" || return 1
    if [[ "$clean_mode" == true ]]; then
      if [[ -n "${log_level_arg}" ]]; then
        _run_build "${py_flag[@]}" "${log_level_arg}" clean "${build_type}"
      else
        _run_build "${py_flag[@]}" clean "${build_type}"
      fi
    else
      if [[ -n "${log_level_arg}" ]]; then
        _run_build "${py_flag[@]}" "${log_level_arg}" "${build_type}"
      else
        _run_build "${py_flag[@]}" "${build_type}"
      fi
    fi
  )
}

# Build command for single package
# Auto-detects package type and builds
mm() {
  local current_dir
  current_dir="$(pwd)"

  if [[ "$current_dir" != "$SROBOTIS_ROOT"/* ]]; then
    echo "[mm] ERROR: Current directory is not within project" >&2
    return 1
  fi

  local is_package=false
  if [[ -f "${current_dir}/CMakeLists.txt" ]] || \
     [[ -f "${current_dir}/package.xml" ]] || \
     [[ -f "${current_dir}/build.sh" ]]; then
    is_package=true
  fi

  if [[ "$is_package" != true ]]; then
    echo "[mm] ERROR: Not a valid package directory" >&2
    echo "[mm] Hint: Run mm from a directory with CMakeLists.txt, package.xml, or build.sh" >&2
    return 1
  fi

  local clean_mode=false
  # Use parent's PARALLEL_JOBS if set (e.g. by m); otherwise nproc
  local parallel_jobs="${PARALLEL_JOBS:-$(nproc)}"
  # For single-package builds, default to verbose so developers can see full output.
  # User can override via -v / --log=quiet|normal|verbose.
  local log_level_arg="--log=verbose"
  # Extra CMake -D options (e.g. -DBUILD_STREAM_DEMO=ON), passed through to cmake configure
  local extra_cmake_args=()
  local py_flag=()
  local deps_mode=""

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -j*)
        parallel_jobs="${1#-j}"
        shift
        ;;
      -v)
        log_level_arg="--log=verbose"
        shift
        ;;
      --log=*)
        log_level_arg="$1"
        shift
        ;;
      --with-deps)
        deps_mode="--with-deps"
        shift
        ;;
      --deps)
        deps_mode="--deps"
        shift
        ;;
      clean)
        clean_mode=true
        shift
        ;;
      -py)
        py_flag=(--py)
        shift
        ;;
      -D*)
        extra_cmake_args+=("$1")
        shift
        ;;
      --)
        shift
        while [[ $# -gt 0 ]]; do
          extra_cmake_args+=("$1")
          shift
        done
        break
        ;;
      -h|--help)
        cat <<EOF
Usage: mm [options] [clean] [--] [-DKEY=VALUE ...]

Build commands (run from package directory):
  mm                   Build current package
  mm -j8               Use 8 parallel jobs
  mm -py               Opt-in: build Python wheel for this package if applicable
  mm -v                Verbose: print full CMake/colcon output to console
  mm --log=LEVEL       Set logging level: quiet|normal|verbose (default: verbose)
  mm --with-deps       Build package dependencies first, then current package
  mm --deps            Build only package dependencies of current package
  mm clean             Clean current package
  mm -DBUILD_STREAM_DEMO=ON    Pass CMake option (CMake packages only)
  mm -- -DOPT1=ON -DOPT2=OFF  Pass multiple CMake options after --

Supported package types:
  - CMake (has CMakeLists.txt)
  - ROS2 (has package.xml)
  - Custom (has build.sh script)
EOF
        return 0
        ;;
      *)
        echo "[mm] ERROR: Unknown option: $1" >&2
        return 1
        ;;
    esac
  done

  if [[ "$clean_mode" == true && -n "${deps_mode}" ]]; then
    echo "[mm] ERROR: clean cannot be combined with ${deps_mode}" >&2
    return 1
  fi

  (
    cd "${SROBOTIS_ROOT}" || return 1
    PARALLEL_JOBS="${parallel_jobs}"
    if [[ ${#extra_cmake_args[@]} -gt 0 ]]; then
      SROBOTIS_CMAKE_EXTRA_ARGS="$(printf '%s\n' "${extra_cmake_args[@]}")"
      export SROBOTIS_CMAKE_EXTRA_ARGS
    fi
    if [[ "$clean_mode" == true ]]; then
      if [[ -n "${log_level_arg}" ]]; then
        _run_build "${py_flag[@]}" "${log_level_arg}" package "${current_dir}" clean
      else
        _run_build "${py_flag[@]}" package "${current_dir}" clean
      fi
    else
      if [[ -n "${log_level_arg}" ]]; then
        _run_build "${py_flag[@]}" "${log_level_arg}" package "${current_dir}" ${deps_mode:+"${deps_mode}"}
      else
        _run_build "${py_flag[@]}" package "${current_dir}" ${deps_mode:+"${deps_mode}"}
      fi
    fi
  )
}

# Convenience function: print a short help message for all helper commands.
# Usage (after sourcing this file):
#   srobot_help
srobot_help() {
  cat <<EOF
[env] Helper commands (SDK build & run):
  lunch [target]               Select build target configuration (interactive menu)
  m [options] [clean]          Build from repo root (all, -C CMake, -R ROS2, -j jobs)
  mm [options] [clean]         Build single package (auto-detect type, supports custom scripts)
  mm --with-deps / --deps      Build current package with deps, or only deps
  m_enable_docker_build        Enable Docker build for this sourced shell
  m_enable_docker_build disable  Disable Docker build for this sourced shell
  m_enable_cross_build         Enable cross build for this sourced shell
  m_enable_cross_build disable Disable cross build for this sourced shell
  venv <python_version>        Create and activate .venv for wheel builds via uv
  m_env_build <app_dir>        Build Python env from pyproject.toml
  m -py / mm -py               Opt-in: pass --py to build.sh for Python wheels after install
  ./build/python_wheels.sh <pkg...>  Build wheels only for given package keys/paths

[env] Helper commands (navigation):
  croot                             cd to repo root
  ccomponent                        cd to components/
  capp                              cd to application/

[env] Helper commands (ROS2 environment):
  sros2_setup                       Prepare ROS2 + SDK overlay in current shell

Examples:
  # Select build target
  lunch           # Show interactive menu to select configuration
  lunch k3-com260-minimal  # Select target directly

  # Build from repo root
  m              # Build all
  m -C           # Build CMake only
  m -R           # Build ROS2 only
  m -j8          # Use 8 parallel jobs
  m -py          # Full build + Python wheels for applicable CMake packages
  m clean        # Clean all

  # Build single package (from package directory)
  mm             # Build current package
  mm -j8         # Use 8 parallel jobs
  mm --with-deps # Build package dependencies first, then current package
  mm --deps      # Build only package dependencies of current package
  mm clean       # Clean current package
  mm -DBUILD_STREAM_DEMO=ON   # Pass CMake options (CMake packages only)

  # Prepare a wheel build environment for Python-enabled components
  venv 3.13.12
  cd vad && mm
EOF
}

echo "[env] SDK_ROOT=${SROBOTIS_ROOT}"
echo "[env] PREFIX=${PREFIX} (install prefix, under ${SROBOTIS_OUTPUT_ROOT})"
echo "[env] ROS_DISTRO=${ROS_DISTRO}"
echo "[env] ROS_SETUP=${ROS_SETUP}"
echo "[env] PATH now includes: ${PREFIX}/bin"
echo "[env] LD_LIBRARY_PATH now includes: ${PREFIX}/lib"
echo "[env] Docker build disabled; run m_enable_docker_build to enable it."
echo "[env] Cross build disabled; run m_enable_cross_build to enable it."
