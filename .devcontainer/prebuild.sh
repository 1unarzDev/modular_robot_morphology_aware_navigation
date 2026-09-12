#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
adapter=linux
if command -v nvidia-smi >/dev/null 2>&1; then
  adapter=nvidia
fi
cp "${script_dir}/docker-compose.override.${adapter}.yml" \
   "${script_dir}/docker-compose.override.yml"
echo "Selected ${adapter} development-container adapter."

