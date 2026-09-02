#!/bin/sh
#
# Split an installed virtualenv into the layers of the runtime image.
#
# The Dockerfile next to this script fills /opt/venv in four steps -- the CARLA
# client, the framework's third-party dependency closure, the framework wheels,
# the scenario wheel -- and calls `capture` after each one.  Everything the step
# added to (or rewrote in) the virtualenv is exported into <export-root>/<name>,
# and the runtime stage lays those trees back on top of one another with one
# COPY, and so one layer, each.  `stack` does the same reassembly on disk, which
# is how the build checks that the layers really do add up to the virtualenv.
#
# The point of the split is the pull rather than the build.  A layer whose
# inputs did not change keeps its digest, so a client that already holds an
# image built for the same CARLA client and framework downloads only the
# scenario layer -- a few hundred kB instead of the whole virtualenv.  For that
# to hold across rebuilds, identical content has to carry identical metadata, so
# every exported file is stamped with LAYER_MTIME instead of the time it
# happened to be installed.  `normalize` re-stamps whatever a later pass (the
# slimming) rewrote.
#
# Usage:
#   venv-layer.sh capture   <venv-dir> <export-root> <layer-name>
#   venv-layer.sh stack     <export-root> <dest-dir>
#   venv-layer.sh normalize <export-root>
#
# State lives in <export-root>/.state, which the runtime stage never copies.

set -eu

#: Stamped on every exported file. 2020-01-01T00:00:00Z; any fixed point does.
#: Overridden by the Dockerfile ARG of the same name, which reaches this script
#: as an environment variable.
: "${LAYER_MTIME:=1577836800}"

usage() {
    cat >&2 <<'USAGE'
Usage: venv-layer.sh capture   <venv-dir> <export-root> <layer-name>
       venv-layer.sh stack     <export-root> <dest-dir>
       venv-layer.sh normalize <export-root>

  capture    Export everything the last install step added to <venv-dir> as
             <export-root>/<layer-name>, then re-baseline for the next step.
  stack      Reassemble the captured layers, in capture order, into <dest-dir>.
  normalize  Re-stamp anything under <export-root> that is not already at
             LAYER_MTIME, so unchanged content keeps its layer digest.
USAGE
    exit 2
}

abspath() {
    (cd "$1" >/dev/null 2>&1 && pwd) \
        || { echo "venv-layer.sh: no such directory: $1" >&2; exit 1; }
}

# Every file and symlink under a directory, as sorted paths relative to it.
# Extra `find` predicates may be passed after the directory.
list_tree() {
    dir=$1
    shift
    (cd "${dir}" && find . -mindepth 1 \( -type f -o -type l \) "$@" -print) \
        | LC_ALL=C sort
}

capture() {
    [ $# -eq 3 ] || usage
    # The listings below run from inside the virtualenv, so every path the
    # script hands to find or cp has to be absolute.
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
        list_tree "${venv_dir}" -newer "${seen}"
    } | LC_ALL=C sort -u > "${state}/delta"

    if [ -s "${state}/delta" ]; then
        # -l: the export is a second *name* for the installed file rather than a
        # second copy, so a captured layer costs its metadata and not its bytes
        # -- and BuildKit records the second name as a link within the same
        # layer diff. -a keeps modes and leaves symlinks as symlinks; --parents
        # rebuilds the tree under the export directory. Falls back to a real
        # copy when the export root is on another filesystem.
        #
        # The alias is why a later step must *replace* a file rather than
        # truncate it in place: an in-place rewrite reaches the copy an earlier
        # layer already exported, and while stacking still yields the new
        # content -- the delta above puts it in the later layer too -- the
        # earlier layer's digest would move. uv replaces, and no path is
        # captured twice in this build.
        (
            cd "${venv_dir}"
            xargs -a "${state}/delta" -d '\n' cp -al --parents -t "${export_root}/${name}" 2>/dev/null \
                || xargs -a "${state}/delta" -d '\n' cp -a --parents -t "${export_root}/${name}"
        )
        # Stamped here, in the step that created these files, rather than in a
        # pass at the end: touching a file that a previous layer wrote copies it
        # up whole, which is the entire virtualenv for the sake of its mtimes.
        find "${export_root}/${name}" -exec touch -h -d "@${LAYER_MTIME}" {} +
    fi

    # `stack` and the runtime image both need the order the layers were taken
    # in, and this is the only place that knows it.
    grep -qxF "${name}" "${state}/order" 2>/dev/null || echo "${name}" >> "${state}/order"

    mv "${state}/now" "${seen}"
    # The next step's files have to look newer than this marker, so it has to
    # carry the time the step ended rather than the time the listing was taken.
    touch "${seen}"

    echo "venv-layer: ${name}: $(wc -l < "${state}/delta" | tr -d ' ') path(s)"
}

stack() {
    [ $# -eq 2 ] || usage
    export_root=$(abspath "$1")
    mkdir -p "$2"
    dest=$(abspath "$2")

    while IFS= read -r layer; do
        [ -n "${layer}" ] || continue
        cp -a "${export_root}/${layer}/." "${dest}/"
    done < "${export_root}/.state/order"
    echo "venv-layer: stacked $(tr '\n' ' ' < "${export_root}/.state/order")into ${dest}"
}

normalize() {
    [ $# -eq 1 ] || usage
    export_root=$(abspath "$1")

    # Only what is not already stamped: on an overlay filesystem every touch
    # copies the file up into the current layer, so re-stamping a file that is
    # already correct would rewrite the virtualenv to change nothing.
    find "${export_root}" -mindepth 1 -path "${export_root}/.state" -prune -o \
        -newermt "@${LAYER_MTIME}" -exec touch -h -d "@${LAYER_MTIME}" {} +
    echo "venv-layer: stamped ${export_root} with @${LAYER_MTIME}"
}

[ $# -ge 1 ] || usage
action=$1
shift
case "${action}" in
    capture) capture "$@" ;;
    stack) stack "$@" ;;
    normalize) normalize "$@" ;;
    -h|--help) usage ;;
    *) echo "venv-layer.sh: unknown command: ${action}" >&2; usage ;;
esac
