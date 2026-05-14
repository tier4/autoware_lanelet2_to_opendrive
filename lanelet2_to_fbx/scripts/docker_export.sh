#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd "${PROJECT_DIR}/.." && pwd)"
IMAGE_TAG="${LL2TOFBX_IMAGE:-ll2tofbx:local}"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e XDG_CONFIG_HOME=/tmp/.config \
  -e XDG_CACHE_HOME=/tmp/.cache \
  -v "${REPO_DIR}:${REPO_DIR}" \
  -w "${REPO_DIR}" \
  "${IMAGE_TAG}" \
  export "$@"
