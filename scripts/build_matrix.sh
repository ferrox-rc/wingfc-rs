#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-all}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${SCRIPT_DIR}/build_matrix.py" "$ACTION" "${@:2}"
