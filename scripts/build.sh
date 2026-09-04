#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run user-friendly builder
exec python3 "${SCRIPT_DIR}/builder.py" "${@}"
