#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_line="source '${workspace_root}/install/setup.bash'"
grep -Fqx "$source_line" "$HOME/.bashrc" 2>/dev/null || \
  printf '\n%s\n' "$source_line" >> "$HOME/.bashrc"

if [[ "${MORPH_NAV_ROSDEP_INSTALL:-0}" == "1" ]]; then
  rosdep install --from-paths "${workspace_root}/src" --ignore-src -y
else
  echo "Set MORPH_NAV_ROSDEP_INSTALL=1 to install package dependencies."
fi

