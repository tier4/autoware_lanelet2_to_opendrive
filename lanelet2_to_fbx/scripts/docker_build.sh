#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_TAG="${LL2TOFBX_IMAGE:-ll2tofbx:local}"

docker build \
  -t "${IMAGE_TAG}" \
  -f "${PROJECT_DIR}/Dockerfile" \
  "${PROJECT_DIR}"
