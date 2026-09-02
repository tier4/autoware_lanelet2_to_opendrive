#!/usr/bin/env bash
#
# Assemble the minimal Docker build context for a generated scenario package.
#
# The context deliberately contains only what the wheel-building stage of the
# Dockerfile next to this script needs -- no VCS history, no tests, no docs,
# no virtualenv -- so that the image build stays fast and nothing that is not
# code ends up in a layer:
#
#     <out>/framework/pyproject.toml        # uv workspace root
#     <out>/framework/<member>/pyproject.toml
#     <out>/framework/<member>/src/
#     <out>/framework/<member>/README.md    # only when the member declares one
#     <out>/framework/carla_wheels/         # local wheels for CARLA releases
#                                           # that are not published to PyPI
#     <out>/scenario/                       # the generated scenario package
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
members=()

usage() {
    cat >&2 <<'USAGE'
Usage: assemble-context.sh --scenario <dir> --out <dir>
                           [--framework <dir>]
                           [--member <rel-path>]...
                           [--carla-wheel-dir <rel-path>]

  --scenario         Generated scenario package (the directory holding its
                     pyproject.toml). Required.
  --out              Directory to assemble the build context in. Created if
                     missing; existing content is removed. Required.
  --framework        uv workspace root providing the framework packages.
                     Defaults to this repository.
  --member           Workspace member to ship, relative to --framework. May be
                     repeated. Defaults to autoware_carla_scenario and
                     autoware_lanelet2_to_opendrive.
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
        --member) members+=("${2:?--member needs a value}"); shift 2 ;;
        --carla-wheel-dir) carla_wheel_dir="${2:?--carla-wheel-dir needs a value}"; shift 2 ;;
        -h|--help) usage ;;
        *) echo "assemble-context.sh: unknown argument: $1" >&2; usage ;;
    esac
done

[ -n "${scenario_dir}" ] || { echo "assemble-context.sh: --scenario is required" >&2; usage; }
[ -n "${out_dir}" ] || { echo "assemble-context.sh: --out is required" >&2; usage; }

if [ ${#members[@]} -eq 0 ]; then
    # autoware_lanelet2_to_opendrive is not a declared dependency of
    # autoware-carla-scenario, but coordinate/road_lanelet_mapping.py imports it
    # at module scope, so the runtime needs it too.
    members=(autoware_carla_scenario autoware_lanelet2_to_opendrive)
fi

abspath() {
    (cd "$1" >/dev/null 2>&1 && pwd) || { echo "assemble-context.sh: no such directory: $1" >&2; exit 1; }
}

scenario_dir="$(abspath "${scenario_dir}")"
framework_dir="$(abspath "${framework_dir}")"

[ -f "${scenario_dir}/pyproject.toml" ] || {
    echo "assemble-context.sh: ${scenario_dir} has no pyproject.toml -- is it a scenario package?" >&2
    exit 1
}
[ -f "${framework_dir}/pyproject.toml" ] || {
    echo "assemble-context.sh: ${framework_dir} has no pyproject.toml -- is it the workspace root?" >&2
    exit 1
}

mkdir -p "${out_dir}"
out_dir="$(abspath "${out_dir}")"

case "${scenario_dir}/" in
    "${out_dir}/"*) echo "assemble-context.sh: --out must not sit inside --scenario" >&2; exit 1 ;;
esac
case "${out_dir}/" in
    "${scenario_dir}/"*) echo "assemble-context.sh: --out must not sit inside --scenario" >&2; exit 1 ;;
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

for member in "${members[@]}"; do
    src="${framework_dir}/${member}"
    [ -f "${src}/pyproject.toml" ] || {
        echo "assemble-context.sh: workspace member '${member}' has no pyproject.toml" >&2
        exit 1
    }
    dst="${out_dir}/framework/${member}"
    mkdir -p "${dst}"
    cp "${src}/pyproject.toml" "${dst}/pyproject.toml"
    # Declared via [project] readme; the build backend fails without it.
    [ -f "${src}/README.md" ] && cp "${src}/README.md" "${dst}/README.md"
    [ -d "${src}/src" ] || {
        echo "assemble-context.sh: workspace member '${member}' has no src/ directory" >&2
        exit 1
    }
    mkdir -p "${dst}/src"
    copy_tree "${src}/src" "${dst}/src"
done

# Always present so the Dockerfile's COPY resolves and the workspace root's
# `[tool.uv] find-links` target exists, even when there is nothing to vendor.
mkdir -p "${out_dir}/framework/carla_wheels"
if [ -d "${framework_dir}/${carla_wheel_dir}" ]; then
    find "${framework_dir}/${carla_wheel_dir}" -maxdepth 1 -name '*.whl' \
        -exec cp {} "${out_dir}/framework/carla_wheels/" \;
fi

copy_tree "${scenario_dir}" "${out_dir}/scenario"

echo "Assembled build context at ${out_dir}"
du -sh "${out_dir}" 2>/dev/null || true
