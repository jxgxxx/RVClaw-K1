#!/usr/bin/env bash
#
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#
# PEP 517 wheel build for SDK packages (python/pyproject.toml or root pyproject.toml).
# - Sourced by build/nonros2.sh: defines srobotis_maybe_build_python_wheel.
# - Executed: ./build/python_wheels.sh <pkg_key_or_dir> [...]  (wheel-only, no full CMake).

: "${REPO_ROOT:=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
: "${OUTPUT_ROOT:=${REPO_ROOT}/output}"
: "${LOG_ROOT:=${OUTPUT_ROOT}/log}"
: "${PREFIX:=${OUTPUT_ROOT}/staging}"

# Usage: srobotis_maybe_build_python_wheel <pkg_key> <pkg_dir> <want_wheel>
#   want_wheel: 1 build wheel when pyproject exists, 0 skip
# Returns 0 on skip or success; non-zero if wheel build failed.
srobotis_maybe_build_python_wheel() {
  local pkg_key="$1"
  local pkg_dir="$2"
  local want_wheel="${3:-0}"

  if [[ "${want_wheel}" != "1" ]]; then
    return 0
  fi

  if [[ "${pkg_key}" == components/thirdparty/* ]]; then
    return 0
  fi

  local wheel_root=""
  if [[ -f "${pkg_dir}/python/pyproject.toml" ]]; then
    wheel_root="${pkg_dir}/python"
  elif [[ -f "${pkg_dir}/pyproject.toml" ]]; then
    wheel_root="${pkg_dir}"
  else
    return 0
  fi

  # Require PyPA pep517 "build" (provides build.__main__). A different top-level
  # package named "build" can make "import build" succeed but "python -m build" fail.
  local py="${SROBOTIS_PYTHON:-${PYTHON:-python3}}"
  if ! "$py" -c "import importlib.util, sys; sys.exit(0 if importlib.util.find_spec('build.__main__') else 1)" >/dev/null 2>&1; then
    echo "[wheel] ERROR: PyPA \`build\` not usable (${py}). Install: sudo apt install -y python3-build" >&2
    return 1
  fi

  # Many native wheels rely on pybind11 via CMake config (find_package(pybind11 CONFIG)).
  if ! dpkg -s pybind11-dev >/dev/null 2>&1; then
    echo "[wheel] ERROR: Missing pybind11-dev. Install: sudo apt install -y pybind11-dev" >&2
    return 1
  fi

  local safe
  safe="$(echo "${pkg_key}" | tr '/ ' '__')"
  local out_dir="${OUTPUT_ROOT}/wheels/${safe}"
  local wheel_log="${LOG_ROOT}/cmake/pkgs/${safe}.wheel.log"

  mkdir -p "${out_dir}" "$(dirname "${wheel_log}")"

  export CMAKE_PREFIX_PATH="${PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  case ":${PKG_CONFIG_PATH:-}:" in
    *":${PREFIX}/lib/pkgconfig:"*) ;;
    *)
      if [[ -n "${PKG_CONFIG_PATH:-}" ]]; then
        export PKG_CONFIG_PATH="${PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH}"
      else
        export PKG_CONFIG_PATH="${PREFIX}/lib/pkgconfig"
      fi
      ;;
  esac
  case ":${LD_LIBRARY_PATH:-}:" in
    *":${PREFIX}/lib:"*) ;;
    *)
      if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
        export LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH}"
      else
        export LD_LIBRARY_PATH="${PREFIX}/lib"
      fi
      ;;
  esac

  echo "[wheel] Building wheel for ${pkg_key} -> ${out_dir}" >&2
  mkdir -p "$(dirname "${wheel_log}")"
  if [[ "${LOG_LEVEL:-quiet}" == "verbose" ]]; then
    if [[ "${LOG_TEE:-1}" == "1" ]]; then
      (
        set -euo pipefail
        cd "${wheel_root}"
        "$py" -m build --wheel --outdir "${out_dir}"
      ) 2>&1 | tee "${wheel_log}"
      return "${PIPESTATUS[0]}"
    fi
    (
      set -euo pipefail
      cd "${wheel_root}"
      "$py" -m build --wheel --outdir "${out_dir}"
    ) >"${wheel_log}" 2>&1 || {
      local rc=$?
      echo "[wheel] ERROR: Wheel build failed for ${pkg_key}. See log: ${wheel_log}" >&2
      tail -n "${LOG_TAIL_LINES:-200}" "${wheel_log}" >&2 || true
      return "${rc}"
    }
    return 0
  fi
  (
    set -euo pipefail
    cd "${wheel_root}"
    "$py" -m build --wheel --outdir "${out_dir}"
  ) >"${wheel_log}" 2>&1 || {
    local rc=$?
    echo "[wheel] ERROR: Wheel build failed for ${pkg_key}. See log: ${wheel_log}" >&2
    tail -n "${LOG_TAIL_LINES:-200}" "${wheel_log}" >&2 || true
    return "${rc}"
  }
  return 0
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  set -euo pipefail
  REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  # shellcheck source=build/common.sh
  source "${REPO_ROOT}/build/common.sh"

  _pw_usage() {
    cat <<EOF
Usage: python_wheels.sh <pkg_key_or_dir> [more ...]

  Each argument is either:
    - a package key under the repo (e.g. components/model_zoo/asr), or
    - an absolute/relative path to a package directory inside the repo.

  Uses PREFIX=\${PREFIX:-${OUTPUT_ROOT}/staging} and writes wheels under:
    \${OUTPUT_ROOT}/wheels/<sanitized_pkg_key>/

  Interpreter: SROBOTIS_PYTHON or PYTHON or python3.
EOF
  }

  _pw_arg_to_pkg_key() {
    local arg="$1"
    if [[ -d "${arg}" ]]; then
      local abs
      abs="$(cd "${arg}" && pwd)"
      if [[ "${abs}" != "${REPO_ROOT}" && "${abs}" != "${REPO_ROOT}/"* ]]; then
        echo "[wheel] ERROR: Directory is not under REPO_ROOT=${REPO_ROOT}: ${abs}" >&2
        return 1
      fi
      if [[ "${abs}" == "${REPO_ROOT}" ]]; then
        echo "[wheel] ERROR: Refusing to treat repo root as a single package." >&2
        return 1
      fi
      echo "${abs#"${REPO_ROOT}/"}"
      return 0
    fi

    if [[ -d "${REPO_ROOT}/${arg}" ]]; then
      echo "${arg}"
      return 0
    fi

    echo "[wheel] ERROR: Not a directory and not a package under repo: ${arg}" >&2
    return 1
  }

  if [[ $# -lt 1 ]]; then
    _pw_usage
    exit 1
  fi

  for arg in "$@"; do
    pkg_key="$(_pw_arg_to_pkg_key "${arg}")" || exit 1
    pkg_dir="${REPO_ROOT}/${pkg_key}"
    echo "[wheel] Package: ${pkg_key}"
    srobotis_maybe_build_python_wheel "${pkg_key}" "${pkg_dir}" 1 || exit 1
  done

  echo "[wheel] Done."
fi
