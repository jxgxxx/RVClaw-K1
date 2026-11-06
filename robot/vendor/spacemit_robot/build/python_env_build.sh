#!/usr/bin/env bash
#
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#

set -euo pipefail

# Generic Python environment builder for a single project.
# Usage (from repo root, or via m_env_build helper):
#   ./build/python_env_build.sh application/ros2/lekiwi/chatbox
#
# Behavior:
#   - Locate pyproject.toml under the given application directory.
#   - Read [project].name as the project name (also used as venv name).
#   - Parse [project].dependencies.
#   - For each dependency:
#       * If there is a local project in this repo whose [project].name
#         matches the dependency package name, install it in editable mode
#         from its source directory (with the same extras as declared).
#       * Otherwise, treat it as a normal third‑party dependency and
#         install it from Python package indexes (e.g. PyPI).
#   - Finally, install the application itself in editable mode:
#       pip install -e <app_dir>
#
# This design:
#   - Does NOT require build/python_projects.json.
#   - Uses only standard pyproject.toml metadata (PEP 621).
#   - Follows the "prefer local source, fall back to remote" principle.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_ROOT="${REPO_ROOT}/output"

usage() {
  cat <<EOF
Usage: python_env_build.sh <app_dir>

  <app_dir>  Path to a Python project directory that contains pyproject.toml,
            e.g. application/ros2/lekiwi/chatbox

Example:
  ./build/python_env_build.sh application/ros2/lekiwi/chatbox
EOF
}

if [[ $# -ne 1 ]]; then
  usage
  exit 1
fi

APP_PATH_RAW="$1"

# Resolve application directory to an absolute path.
if [[ "${APP_PATH_RAW}" = /* ]]; then
  APP_DIR="${APP_PATH_RAW}"
else
  APP_DIR="${REPO_ROOT}/${APP_PATH_RAW}"
fi

if [[ ! -d "${APP_DIR}" ]]; then
  echo "[env] Application directory does not exist: ${APP_DIR}" >&2
  exit 1
fi

PYPROJECT_FILE="${APP_DIR}/pyproject.toml"
if [[ ! -f "${PYPROJECT_FILE}" ]]; then
  echo "[env] pyproject.toml not found in: ${APP_DIR}" >&2
  exit 1
fi

#
# Use Python to:
#   - read the application's pyproject.toml
#   - discover all other local projects (by scanning for pyproject.toml)
#   - classify dependencies into:
#       * LOCAL  -> must be installed from local source in editable mode (with --no-deps)
#       * REMOTE -> third‑party deps to install from indexes
#   - For LOCAL projects, also read their own dependencies and optional‑dependencies
#     for the extras actually requested by the application, and add those to REMOTE,
#     excluding anything that is itself a LOCAL project (so local SDKs are never
#     pulled from PyPI).
# Output format (tab‑separated lines):
#   APP    <project_name>  <app_dir>
#   LOCAL  <pkg_name>      <src_dir>  <extras_string_or_empty>
#   REMOTE <original_dependency_spec>
mapfile -t META_LINES < <(python3 - "${REPO_ROOT}" "${APP_DIR}" << 'PY'
import re
import sys
from pathlib import Path

try:
    import tomllib  # Python 3.11+
except ImportError:  # pragma: no cover
    print("tomllib is required (Python 3.11+).", file=sys.stderr)
    sys.exit(1)


def load_pyproject(path: Path):
    try:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except Exception as e:  # pragma: no cover
        print(f"[env] Failed to read {path}: {e}", file=sys.stderr)
        sys.exit(1)
    return data


repo_root = Path(sys.argv[1])
app_dir = Path(sys.argv[2])
pyproject_file = app_dir / "pyproject.toml"

py_data = load_pyproject(pyproject_file)
project = py_data.get("project") or {}
app_name = project.get("name")
if not app_name:
    print(f"[env] Missing [project].name in {pyproject_file}", file=sys.stderr)
    sys.exit(1)

app_deps = project.get("dependencies") or []

# Discover local projects by scanning for pyproject.toml and reading
# their [project].name. This keeps everything standards‑based and avoids
# extra manifest files.
local_projects: dict[str, Path] = {}
local_py_data: dict[str, dict] = {}

for py in repo_root.rglob("pyproject.toml"):
    data = load_pyproject(py)
    proj = data.get("project") or {}
    name = proj.get("name")
    if not name:
        continue
    # We allow later entries to override earlier ones; in this repo layout
    # project names are expected to be unique.
    local_projects[name] = py.parent
    local_py_data[name] = data


# Helper to extract the base package name and extras from a dependency string.
# Examples:
#   "pipecat-ai[local,mcp]"      -> ("pipecat-ai", ["local", "mcp"])
#   "python-dotenv>=1.0,<2.0"    -> ("python-dotenv", [])
name_regex = re.compile(r"^[A-Za-z0-9_.-]+")
extras_regex = re.compile(r"\[([^\]]+)\]")


def parse_dep(dep: str):
    stripped = dep.strip()
    if not stripped:
        return None, [], stripped

    m = name_regex.match(stripped)
    if not m:
        return None, [], stripped

    base_name = m.group(0)
    extras_match = extras_regex.search(stripped)
    extras = []
    if extras_match:
        extras = [e.strip() for e in extras_match.group(1).split(",") if e.strip()]

    return base_name, extras, stripped


required_local: dict[str, set[str]] = {}
remote_deps: set[str] = set()

# Step 1: classify direct application dependencies into LOCAL vs REMOTE.
for dep in app_deps:
    if not isinstance(dep, str):
        continue
    base_name, extras, original = parse_dep(dep)
    if not base_name:
        # Cannot parse base name reliably; treat as remote.
        remote_deps.add(original)
        continue

    if base_name in local_projects:
        # Application depends on a local project.
        current = required_local.setdefault(base_name, set())
        current.update(extras)
    else:
        # Normal third‑party dependency.
        remote_deps.add(original)


# Step 2: for each required LOCAL project, collect its own dependencies and
#         optional‑dependencies for the extras that were explicitly requested
#         by the application. All of these go into the "candidate remote deps"
#         set; we will later filter out anything that is itself a local project.
for name, extras in required_local.items():
    data = local_py_data.get(name)
    if not data:
        continue
    proj = data.get("project") or {}

    # Base dependencies of the local project.
    for dep in proj.get("dependencies") or []:
        if isinstance(dep, str):
            remote_deps.add(dep.strip())

    # Optional dependencies for the extras used in the application.
    opt = proj.get("optional-dependencies") or {}
    for extra in extras:
        for dep in opt.get(extra, []) or []:
            if isinstance(dep, str):
                remote_deps.add(dep.strip())


# Step 3: emit APP / LOCAL / REMOTE lines.
print(f"APP\t{app_name}\t{app_dir}")

for name, extras in sorted(required_local.items()):
    src_dir = local_projects.get(name)
    extras_str = ",".join(sorted(extras)) if extras else ""
    print(f"LOCAL\t{name}\t{src_dir}\t{extras_str}")

for dep in sorted(remote_deps):
    base_name, _extras, original = parse_dep(dep)
    # If the base name corresponds to a local project, we never install it
    # from indexes; it will be installed from source in editable mode.
    if base_name and base_name in local_projects:
        continue
    print(f"REMOTE\t{original}")
PY
)

APP_NAME=""
APP_DIR_RESOLVED=""
LOCAL_PKGS=()
LOCAL_DIRS=()
LOCAL_EXTRAS=()
REMOTE_DEPS=()

for line in "${META_LINES[@]}"; do
  kind="${line%%$'\t'*}"
  rest="${line#*$'\t'}"
  case "${kind}" in
    APP)
      IFS=$'\t' read -r APP_NAME APP_DIR_RESOLVED <<< "${rest}"
      ;;
    LOCAL)
      IFS=$'\t' read -r pkg_name src_dir extras <<< "${rest}"
      LOCAL_PKGS+=("${pkg_name}")
      LOCAL_DIRS+=("${src_dir}")
      LOCAL_EXTRAS+=("${extras}")
      ;;
    REMOTE)
      REMOTE_DEPS+=("${rest}")
      ;;
  esac
done

if [[ -z "${APP_NAME}" || -z "${APP_DIR_RESOLVED}" ]]; then
  echo "[env] Failed to resolve application metadata from pyproject.toml." >&2
  exit 1
fi

APP_DIR="${APP_DIR_RESOLVED}"

ENV_DIR="${OUTPUT_ROOT}/envs/${APP_NAME}"
mkdir -p "${OUTPUT_ROOT}/envs"

if [[ ! -d "${ENV_DIR}" ]]; then
  echo "[env] Creating virtualenv at ${ENV_DIR}"
  python3 -m venv "${ENV_DIR}"
else
  echo "[env] Using existing virtualenv at ${ENV_DIR}"
fi

# shellcheck source=/dev/null
source "${ENV_DIR}/bin/activate"

python3 -m pip install --upgrade pip

if [[ ${#LOCAL_PKGS[@]} -gt 0 ]]; then
  echo "[env] Uninstalling any existing local project distributions: ${LOCAL_PKGS[*]}"
  python3 -m pip uninstall -y "${LOCAL_PKGS[@]}" >/dev/null 2>&1 || true

  echo "[env] Installing local projects in editable mode (from source, no deps):"
  for idx in "${!LOCAL_PKGS[@]}"; do
    name="${LOCAL_PKGS[$idx]}"
    src_dir="${LOCAL_DIRS[$idx]}"
    extras="${LOCAL_EXTRAS[$idx]}"

    if [[ ! -d "${src_dir}" ]]; then
      echo "[env] WARNING: Local project directory not found for ${name}: ${src_dir}, skipping." >&2
      continue
    fi

    if [[ -n "${extras}" ]]; then
      echo "       - ${name} (from ${src_dir}, extras=[${extras}])"
      python3 -m pip install -e "${src_dir}[${extras}]" --no-deps
    else
      echo "       - ${name} (from ${src_dir})"
      python3 -m pip install -e "${src_dir}" --no-deps
    fi
  done
else
  echo "[env] No local projects detected in dependencies."
fi

echo "[env] Installing application '${APP_NAME}' in editable mode from ${APP_DIR} (no deps, since we handle them separately)"
python3 -m pip install -e "${APP_DIR}" --no-deps

if [[ ${#REMOTE_DEPS[@]} -gt 0 ]]; then
  echo "[env] Installing remaining third‑party dependencies from pyproject.toml:"
  printf '       %s\n' "${REMOTE_DEPS[@]}"
  python3 -m pip install "${REMOTE_DEPS[@]}"
else
  echo "[env] No third‑party dependencies to install from pyproject.toml."
fi

echo
echo "[env] Python environment for project '${APP_NAME}' is ready."
echo "[env] To use it in a shell, run:"
echo "       source ${ENV_DIR}/bin/activate"
echo "       # then run your CLI from [project.scripts] in ${APP_DIR}/pyproject.toml"


