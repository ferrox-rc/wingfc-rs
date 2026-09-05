#!/usr/bin/env bash
# ==============================================================================
# WingFC-rs Automated Release Script
# Leverages cargo-workspace, cargo-release, and git-cliff for semver & changelog
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT_DIR}"

if ! command -v cargo-release &>/dev/null && ! cargo release --version &>/dev/null; then
    echo "ERROR: 'cargo-release' is not found."
    echo "Install it via: cargo install cargo-release (or place in PATH)"
    exit 1
fi

if ! command -v git-cliff &>/dev/null; then
    echo "ERROR: 'git-cliff' is not found."
    echo "Install it via: cargo install git-cliff (or place in PATH)"
    exit 1
fi

LEVEL="${1:-patch}"

echo "======================================================================"
echo "WingFC-rs Automated Release: ${LEVEL}"
echo "======================================================================"

# Ensure working directory is clean
if [[ -n $(git status --porcelain) ]]; then
    echo "ERROR: Working tree is dirty. Please commit or stash changes first."
    git status -s
    exit 1
fi

# Run cargo release
cargo release "${LEVEL}" --execute --no-confirm

echo "Release ${LEVEL} completed successfully!"
