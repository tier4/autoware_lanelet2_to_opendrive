#!/usr/bin/env bash
#
# Assemble the minimal Docker build context for a generated scenario package.
#
# The context carries only what the wheel-building stage of the Dockerfile next
# to this script reads:
#
#     <out>/framework/pyproject.toml        # uv workspace root
#     <out>/framework/uv.lock               # pins every framework dependency
#     <out>/framework/<member>/pyproject.toml
#     <out>/framework/<member>/src/
#     <out>/framework/<member>/README.md    # only when the member declares one
#     <out>/framework/carla_wheels/         # local wheels for CARLA releases
#                                           # that are not published to PyPI
#     <out>/scenario/                       # the generated scenario package,
#                                           # minus VCS, caches and build output
#
# Workspace members are discovered rather than listed, so the set stays in step
# with `[tool.uv.workspace] members` without this script having to parse TOML.
#
# Used by action.yml next to it, and by anyone building the image by hand
# (see autoware_carla_scenario/docs/docker.md).

set -euo pipefail

# .github/actions/pack-scenario-image -> repository root.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

scenario_dir=""
out_dir=""
framework_dir="${REPO_ROOT}"
carla_wheel_dir="carla_wheels"

usage() {
    cat >&2 <<'USAGE'
Usage: assemble-context.sh --scenario <dir> --out <dir>
                           [--framework <dir>]
                           [--carla-wheel-dir <rel-path>]

  --scenario         Generated scenario package (the directory holding its
                     pyproject.toml). Required.
  --out              Directory to assemble the build context in. Created if
                     missing; existing content is removed. Required.
  --framework        uv workspace root providing the framework packages.
                     Defaults to this repository.
  --carla-wheel-dir  Directory of local CARLA wheels, relative to --framework.
                     Defaults to carla_wheels. Missing or empty is fine: the
                     client is then resolved from PyPI.
USAGE
    exit 2
}

while [ $# -gt 0 ]; do
    case "$1" in
        --scenario) scenario_dir="${2:?--scenario needs a value}"; shift 2 ;;
        --out) out_dir="${2:?--out needs a value}"; shift 2 ;;
        --framework) framework_dir="${2:?--framework needs a value}"; shift 2 ;;
        --carla-wheel-dir) carla_wheel_dir="${2:?--carla-wheel-dir needs a value}"; shift 2 ;;
        -h|--help) usage ;;
        *) echo "assemble-context.sh: unknown argument: $1" >&2; usage ;;
    esac
done

[ -n "${scenario_dir}" ] || { echo "assemble-context.sh: --scenario is required" >&2; usage; }
[ -n "${out_dir}" ] || { echo "assemble-context.sh: --out is required" >&2; usage; }

abspath() {
    (cd "$1" >/dev/null 2>&1 && pwd) || { echo "assemble-context.sh: no such directory: $1" >&2; exit 1; }
}

scenario_dir="$(abspath "${scenario_dir}")"
framework_dir="$(abspath "${framework_dir}")"

[ -f "${scenario_dir}/pyproject.toml" ] || {
    echo "assemble-context.sh: ${scenario_dir} has no pyproject.toml -- point it at the directory 'scenario-new' created." >&2
    exit 1
}
[ -f "${framework_dir}/pyproject.toml" ] || {
    echo "assemble-context.sh: ${framework_dir} has no pyproject.toml -- is it the workspace root?" >&2
    exit 1
}

mkdir -p "${out_dir}"
out_dir="$(abspath "${out_dir}")"

case "${out_dir}/:${scenario_dir}/" in
    "${scenario_dir}/"*|*":${out_dir}/"*)
        echo "assemble-context.sh: --out and --scenario must not contain one another" >&2
        exit 1
        ;;
esac

rm -rf "${out_dir:?}"/framework "${out_dir:?}"/scenario
mkdir -p "${out_dir}/framework" "${out_dir}/scenario"

# Copy a source tree while dropping everything the build does not read.
copy_tree() {
    tar -C "$1" \
        --exclude=.git \
        --exclude=.venv \
        --exclude=.mypy_cache \
        --exclude=.pytest_cache \
        --exclude=.ruff_cache \
        --exclude=__pycache__ \
        --exclude='*.pyc' \
        --exclude=dist \
        --exclude='*.egg-info' \
        -cf - . | tar -C "$2" -xf -
}

cp "${framework_dir}/pyproject.toml" "${out_dir}/framework/pyproject.toml"
# Lets the wheelhouse stage export a constraints file, so the image pins every
# framework dependency and not just the CARLA client.
cp "${framework_dir}/uv.lock" "${out_dir}/framework/uv.lock"

# Every workspace member looks the same from here: a directory with a
# pyproject.toml and a src/. Discovering them keeps this in step with
# `[tool.uv.workspace] members` without parsing TOML in shell; `uv build
# --all-packages` in the Dockerfile then builds whatever was copied.
members=0
for candidate in "${framework_dir}"/*/; do
    [ -f "${candidate}pyproject.toml" ] && [ -d "${candidate}src" ] || continue
    dst="${out_dir}/framework/$(basename "${candidate}")"
    mkdir -p "${dst}/src"
    cp "${candidate}pyproject.toml" "${dst}/pyproject.toml"
    # Declared via [project] readme; the build backend fails without it.
    [ -f "${candidate}README.md" ] && cp "${candidate}README.md" "${dst}/README.md"
    copy_tree "${candidate}src" "${dst}/src"
    members=$((members + 1))
done
[ "${members}" -gt 0 ] || {
    echo "assemble-context.sh: no workspace members found under ${framework_dir}" >&2
    exit 1
}

# Always present so the Dockerfile's COPY resolves and the workspace root's
# `[tool.uv] find-links` target exists, even when there is nothing to vendor.
mkdir -p "${out_dir}/framework/carla_wheels"
if [ -d "${framework_dir}/${carla_wheel_dir}" ]; then
    find "${framework_dir}/${carla_wheel_dir}" -maxdepth 1 -name '*.whl' \
        -exec cp -t "${out_dir}/framework/carla_wheels/" {} +
fi

copy_tree "${scenario_dir}" "${out_dir}/scenario"

echo "Assembled build context at ${out_dir} (${members} workspace member(s))"
du -sh "${out_dir}" 2>/dev/null || true
