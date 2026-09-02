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
#     <out>/slim-venv.py                    # run by the venv stage
#
# Members come from `[tool.uv.workspace] members` in the root manifest, globs
# and all, so the set stays in step with the workspace itself.
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
# The wheelhouse stage exports this as a constraints file, so the image pins
# every framework dependency and not just the CARLA client.
[ -f "${framework_dir}/uv.lock" ] || {
    echo "assemble-context.sh: ${framework_dir} has no uv.lock -- run 'uv lock' first; the image pins its dependencies from it" >&2
    exit 1
}
cp "${framework_dir}/uv.lock" "${out_dir}/framework/uv.lock"

# The root manifest is the authority on which members exist: its `members`
# entries are globs relative to the workspace root, and may name nested paths
# such as "packages/*". Getting this wrong is not loud -- `uv build
# --all-packages` skips a member that is not on disk without a word -- so a
# member missed here would ship an image quietly lacking that wheel.
resolve_declared_members() {
    local interpreter
    for interpreter in python3 python3.13 python3.12 python3.11; do
        command -v "${interpreter}" >/dev/null 2>&1 || continue
        "${interpreter}" -c 'import tomllib' >/dev/null 2>&1 || continue
        "${interpreter}" - "${framework_dir}" <<'PY'
import sys
import tomllib
from pathlib import Path

root = Path(sys.argv[1])
with (root / "pyproject.toml").open("rb") as handle:
    workspace = tomllib.load(handle).get("tool", {}).get("uv", {}).get("workspace", {})

excluded = {path for pattern in workspace.get("exclude", []) for path in root.glob(pattern)}
seen: list[Path] = []
for pattern in workspace.get("members", []):
    for path in sorted(root.glob(pattern)):
        if path.is_dir() and path not in excluded and path not in seen:
            seen.append(path)
for path in seen:
    print(path.relative_to(root))
PY
        return 0
    done
    return 1
}

declare -a member_paths=()
if declared="$(resolve_declared_members)"; then
    while IFS= read -r line; do
        [ -n "${line}" ] && member_paths+=("${line}")
    done <<< "${declared}"
else
    # No interpreter new enough for tomllib (3.11+). Fall back to the shape a
    # uv workspace has in practice -- a depth-one directory with a pyproject --
    # and say so, because this cannot see a nested member.
    echo "assemble-context.sh: no python3 with tomllib; falling back to a depth-one scan" >&2
    for candidate in "${framework_dir}"/*/; do
        [ -f "${candidate}pyproject.toml" ] || continue
        member_paths+=("$(basename "${candidate}")")
    done
fi

[ ${#member_paths[@]} -gt 0 ] || {
    echo "assemble-context.sh: no workspace members found under ${framework_dir}" >&2
    exit 1
}

for member in "${member_paths[@]}"; do
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
    if [ -d "${src}/src" ]; then
        mkdir -p "${dst}/src"
        copy_tree "${src}/src" "${dst}/src"
    else
        # A member that does not use a src/ layout: take the whole tree, the
        # way the scenario package is taken.
        copy_tree "${src}" "${dst}"
    fi
done

# Always present so the Dockerfile's COPY resolves and the workspace root's
# `[tool.uv] find-links` target exists, even when there is nothing to vendor.
mkdir -p "${out_dir}/framework/carla_wheels"
if [ -d "${framework_dir}/${carla_wheel_dir}" ]; then
    find "${framework_dir}/${carla_wheel_dir}" -maxdepth 1 -name '*.whl' \
        -exec cp -t "${out_dir}/framework/carla_wheels/" {} +
fi

copy_tree "${scenario_dir}" "${out_dir}/scenario"

# The Dockerfile COPYs this, so it has to live in the context.
cp "$(dirname "${BASH_SOURCE[0]}")/slim-venv.py" "${out_dir}/slim-venv.py"

echo "Assembled build context at ${out_dir} (${#member_paths[@]} workspace member(s): ${member_paths[*]})"
du -sh "${out_dir}" 2>/dev/null || true
