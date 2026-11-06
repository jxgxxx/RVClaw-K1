#!/usr/bin/env bash
#
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#

# This file is meant to be SOURCED by build scripts (e.g. build/build.sh).
# It provides jq-based helpers for:
# - reading target/*.json (enabled_packages, enabled_package_options, options)
# - reading per-package package.json metadata

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[jq] ERROR: This script must be sourced, not executed." >&2
  exit 1
fi

# ----------------------------------------------------------------------------
# jq availability
# ----------------------------------------------------------------------------

has_jq() {
  command -v jq >/dev/null 2>&1
}

# ----------------------------------------------------------------------------
# Per-package package.json helpers
# ----------------------------------------------------------------------------

read_package_json_deps() {
  local pkg_json="$1"
  if ! has_jq || [[ ! -f "${pkg_json}" ]]; then
    return 0
  fi
  jq -r '.dependencies[]? // empty' "${pkg_json}" 2>/dev/null || true
}

read_package_json_sysdeps_lines() {
  local pkg_json="$1"
  if ! has_jq || [[ ! -f "${pkg_json}" ]]; then
    return 0
  fi
  jq -r '
    (.system_dependencies.required[]? | "required|\(.name)|\(.check)") ,
    (.system_dependencies.optional[]? | "optional|\(.name)|\(.check)")
  ' "${pkg_json}" 2>/dev/null || true
}

read_package_json_option_list() {
  local pkg_json="$1"
  local jq_expr="$2"
  if ! has_jq || [[ ! -f "${pkg_json}" ]]; then
    return 0
  fi
  jq -r "${jq_expr}" "${pkg_json}" 2>/dev/null || true
}

# ----------------------------------------------------------------------------
# Target JSON helpers
#
# Required variables in the sourcing scope:
# - BUILD_CONFIG_FILE: path to selected target/<name>.json
# ----------------------------------------------------------------------------

# Get enabled packages from config.
# Supports two formats:
# 1) New: .enabled_packages = ["components/..", "middleware/..", ...]
# 2) Legacy: .packages map with .enabled bool
get_enabled_packages_any() {
  local config_file="${BUILD_CONFIG_FILE:-}"
  if [[ -z "${config_file}" || ! -f "${config_file}" ]] || ! has_jq; then
    return 0
  fi

  # Prefer new format when present and non-empty
  local has_new
  has_new="$(jq -r '(.enabled_packages // []) | length' "${config_file}" 2>/dev/null || echo 0)"
  if [[ "${has_new}" != "0" ]]; then
    jq -r '.enabled_packages[]? // empty' "${config_file}" 2>/dev/null || true
    return 0
  fi

  # Fallback to legacy format
  local category="$1"  # optional filter: components|middleware|application, empty = all
  if [[ -n "${category}" ]]; then
    jq -r --arg cat "${category}" '
      .packages | to_entries[] |
      select(.key | startswith($cat + "/")) |
      select(.value.enabled // false == true) |
      .key
    ' "${config_file}" 2>/dev/null || true
  else
    jq -r '
      .packages | to_entries[] |
      select(.value.enabled // false == true) |
      .key
    ' "${config_file}" 2>/dev/null || true
  fi
}

# Read package options from config (new format only).
# Example:
#   get_target_option_list "components/peripherals/motor" '.enabled_drivers[]? // empty'
get_target_option_list() {
  local pkg_key="$1"
  local jq_expr="$2"
  local config_file="${BUILD_CONFIG_FILE:-}"
  if [[ -z "${config_file}" || ! -f "${config_file}" ]] || ! has_jq; then
    return 0
  fi
  jq -r --arg pkg "${pkg_key}" "
    (.enabled_package_options[\$pkg] // {}) | ${jq_expr}
  " "${config_file}" 2>/dev/null || true
}

# Compute transitive closure of dependencies starting from an initial enabled list.
# Outputs the full list (one per line).
resolve_enabled_with_metadata() {
  local config_file="${BUILD_CONFIG_FILE:-}"
  if ! has_jq || [[ -z "${config_file}" || ! -f "${config_file}" ]]; then
    return 0
  fi

  local queue=()
  local seen=()

  mapfile -t queue < <(get_enabled_packages_any "")

  while [[ ${#queue[@]} -gt 0 ]]; do
    local pkg="${queue[0]}"
    queue=("${queue[@]:1}")
    [[ -z "${pkg}" ]] && continue

    local already=false
    for s in "${seen[@]}"; do
      [[ "${s}" == "${pkg}" ]] && { already=true; break; }
    done
    [[ "${already}" == true ]] && continue

    seen+=("${pkg}")

    while IFS= read -r dep; do
      [[ -z "${dep}" ]] && continue
      queue+=("${dep}")
    done < <(read_package_deps "${pkg}")
  done

  printf '%s\n' "${seen[@]}"
}

# Read optional parallel_jobs from target json.
target_parallel_jobs() {
  local config_file="${BUILD_CONFIG_FILE:-}"
  if ! has_jq || [[ -z "${config_file}" || ! -f "${config_file}" ]]; then
    return 0
  fi
  jq -r '.options.parallel_jobs // empty' "${config_file}" 2>/dev/null || true
}


