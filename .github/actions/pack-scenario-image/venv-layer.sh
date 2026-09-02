#!/bin/sh
#
# Split an installed virtualenv into the layers of the runtime image.
#
# The Dockerfile next to this script fills /opt/venv in four steps -- the CARLA
# client, the framework's third-party dependency closure, the framework wheels,
# the scenario wheel -- and calls `capture` after each one.  Everything the step
# added to (or rewrote in) the virtualenv is copied out into
# <export-root>/<name>, and the runtime stage lays those trees back on top of
# one another with one COPY, and so one layer, each.
#
# The point of the split is the pull rather than the build.  A layer whose
# inputs did not change keeps its digest, so a client that already holds an
# image built for the same CARLA client and framework downloads only the
# scenario layer -- a few hundred kB instead of the whole virtualenv.
#
# `normalize` is what makes that hold across rebuilds: it stamps every exported
# file with one fixed timestamp, so identical content lands in an identical
# layer even though the files were installed months apart.
#
# Usage:
#   venv-layer.sh capture   <venv-dir> <export-root> <layer-name>
#   venv-layer.sh normalize <export-root> [epoch-seconds]
#
# State lives in <export-root>/.state, which the runtime stage never copies.

set -eu

#: 2020-01-01T00:00:00Z. Any fixed point does; a timestamp inside the range
#: every archive format can represent avoids surprises in tooling downstream.
DEFAULT_EPOCH=1577836800

usage() {
    cat >&2 <<'USAGE'
Usage: venv-layer.sh capture   <venv-dir> <export-root> <layer-name>
       venv-layer.sh normalize <export-root> [epoch-seconds]

  capture    Copy everything the last install step added to <venv-dir> into
             <export-root>/<layer-name>, then re-baseline for the next step.
  normalize  Stamp every file under <export-root> with a fixed timestamp so
             unchanged content produces an unchanged layer.
USAGE
    exit 2
}

abspath() {
    (cd "$1" >/dev/null 2>&1 && pwd) \
        || { echo "venv-layer.sh: no such directory: $1" >&2; exit 1; }
}

# Every file and symlink in the virtualenv, as paths relative to its root.
list_tree() {
    (cd "$1" && find . -mindepth 1 \( -type f -o -type l \) -print) | LC_ALL=C sort
}

capture() {
    [ $# -eq 3 ] || usage
    # The listings below run from inside the virtualenv, so every path the
    # script hands to find or tar has to be absolute.
    venv_dir=$(abspath "$1")
    mkdir -p "$2"
    export_root=$(abspath "$2")
    name=$3

    state="${export_root}/.state"
    seen="${state}/seen"
    mkdir -p "${state}" "${export_root}/${name}"
    # First call: nothing has been captured, and the whole virtualenv is newer
    # than the epoch, so both halves of the delta below agree on "everything".
    [ -e "${seen}" ] || touch -d @0 "${seen}"

    list_tree "${venv_dir}" > "${state}/now"

    # A file belongs to this step if it was not there before, or if the step
    # rewrote it in place.  The path diff catches the first even for an
    # installer that copies timestamps out of the wheel it unpacks; -newer
    # catches the second, which no path diff can see.
    {
        LC_ALL=C comm -13 "${seen}" "${state}/now"
        (cd "${venv_dir}" && find . -mindepth 1 \( -type f -o -type l \) -newer "${seen}" -print)
    } | LC_ALL=C sort -u > "${state}/delta"

    if [ -s "${state}/delta" ]; then
        # --no-recursion: the list is already every file, and a directory named
        # in it would otherwise drag in the files of a later layer.  Missing
        # parent directories are created on extraction.
        #
        # `set -e` covers the extracting tar, but the reading one only reports
        # through a pipe -- and the shell in the image is dash, which has no
        # pipefail -- so its failure is recorded by hand.  A half-copied layer
        # must not pass for a complete one.
        rm -f "${state}/read-failed"
        { tar -C "${venv_dir}" --no-recursion -T "${state}/delta" -cf - \
            || echo failed > "${state}/read-failed"; } \
            | tar -C "${export_root}/${name}" -xf -
        if [ -e "${state}/read-failed" ]; then
            echo "venv-layer.sh: reading ${venv_dir} failed" >&2
            exit 1
        fi
    fi

    mv "${state}/now" "${seen}"
    # The next step's files have to look newer than this marker, so it has to
    # carry the time the step ended rather than the time the listing was taken.
    touch "${seen}"

    echo "venv-layer: ${name}: $(wc -l < "${state}/delta" | tr -d ' ') path(s)"
}

normalize() {
    [ $# -ge 1 ] || usage
    export_root=$(abspath "$1")
    epoch=${2:-${DEFAULT_EPOCH}}

    find "${export_root}" -mindepth 1 -path "${export_root}/.state" -prune -o \
        -exec touch -h -d "@${epoch}" {} +
    echo "venv-layer: stamped ${export_root} with @${epoch}"
}

[ $# -ge 1 ] || usage
action=$1
shift
case "${action}" in
    capture) capture "$@" ;;
    normalize) normalize "$@" ;;
    -h|--help) usage ;;
    *) echo "venv-layer.sh: unknown command: ${action}" >&2; usage ;;
esac
