"""Top-level test configuration for autoware_carla_scenario.

Generates derived test data (e.g. OpenDRIVE files) on-demand so that
CI and local development work without committing large generated files.
"""

from __future__ import annotations

import hashlib
import os
import subprocess
from pathlib import Path

import pytest
from filelock import FileLock

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_CONVERTER_TEST_DATA = (
    _PROJECT_ROOT / "autoware_lanelet2_to_opendrive" / "test" / "data"
)
_XODR_PATH = _CONVERTER_TEST_DATA / "nishishinjuku_carla.xodr"
_OSM_PATH = _CONVERTER_TEST_DATA / "nishishinjuku.osm"

# Sidecar recording which inputs produced the XODR sitting at ``_XODR_PATH``.
# See :func:`_input_fingerprint` for what goes into it and why.
_STAMP_PATH = _XODR_PATH.with_name(_XODR_PATH.name + ".inputs")

_CONVERTER_SRC = (
    _PROJECT_ROOT
    / "autoware_lanelet2_to_opendrive"
    / "src"
    / "autoware_lanelet2_to_opendrive"
)

# Bump when the *shape* of the fingerprint changes (a new input is added, the
# hash is computed differently). Every existing stamp then mismatches and the
# fixture regenerates, which is the intended effect: a stamp written by an
# older scheme makes no claim about the newer one.
_FINGERPRINT_SCHEMA = b"v1"

# Extensions under the converter source tree that can change the emitted XODR.
# ``.yaml`` matters as much as ``.py`` here: the conversion is Hydra-driven and
# ``conf/map/nishishinjuku.yaml`` / ``conf/target/carla.yaml`` carry the
# preprocessing operations, the ParamPoly3 segmentation and the width sampling.
_FINGERPRINT_SUFFIXES = frozenset({".py", ".yaml", ".yml"})

# The conversion invocation, minus the two paths that vary per run. Kept as a
# module constant so the fingerprint and the subprocess cannot drift apart:
# changing the target or the map selection here invalidates every stamp.
_CONVERT_ARGV = ("uv", "run", "convert", "map=nishishinjuku", "target=carla")


def _input_fingerprint(
    osm_path: Path | None = None,
    src_root: Path | None = None,
    argv: tuple[str, ...] | None = None,
) -> str:
    """Hash everything that determines the content of the generated XODR.

    The fixture below used to key its cache on nothing at all -- the artifact
    existing was taken to mean the artifact was current.  It is not: the XODR
    is *derived*, and a checkout that changes the converter or its Hydra
    configuration changes what the converter would emit.  A stale artifact
    then gets inspected by tests written against the new behaviour, and the
    failure surfaces somewhere unrelated -- a lane count, a contact point, a
    geometry length -- with nothing pointing back at the cache.  Three
    separate debugging sessions have been spent on exactly that.

    The inputs are:

    * ``nishishinjuku.osm`` -- the source map.
    * every ``.py`` / ``.yaml`` / ``.yml`` under the converter's source tree,
      which covers both the conversion code and ``conf/`` (the map and target
      configs are what set the preprocessing operations, the ParamPoly3
      segmentation and the width sampling).
    * the exact ``convert`` argv, so changing the invocation invalidates too.

    Paths are hashed alongside contents, so adding or deleting a file changes
    the fingerprint even when no surviving file changed.  Files are visited in
    sorted order, so the result does not depend on directory iteration order.

    This deliberately does *not* consult git: the fingerprint has to work in a
    source checkout with uncommitted edits, which is the case that bites, and
    an installed sdist has no git metadata at all.

    Cost, measured in the ``dev`` container at ``818e8a00``: **587 ms** per
    call, over 59 source files plus the 10.6 MB OSM -- the OSM read dominates,
    and it is slower still across a bind mount.  It runs once per session.

    The cost that matters is the other one: touching any converter source file
    forces one reconversion on the next test run, measured at **~90 s**.  An
    unchanged tree costs nothing (a no-op run of a single test completes in
    0.8 s).  That is the intended trade -- 90 s once after a source change, in
    exchange for the artifact meaning what it says.

    Note that the emitted XODR is **not byte-reproducible**: its ``<header>``
    carries a wall-clock ``date`` attribute, so two conversions of identical
    inputs differ.  Comparing XODR hashes therefore tells you the file was
    rewritten, not that its content changed -- do not use one as a proxy for
    the other.

    The three arguments exist so the mechanism can be exercised against a
    synthetic tree (see ``test_generated_fixture_invalidation.py``) without
    mutating the real source checkout, which under ``-n`` would be visible to
    every other worker.  Production callers pass none of them.
    """
    osm_path = _OSM_PATH if osm_path is None else osm_path
    src_root = _CONVERTER_SRC if src_root is None else src_root
    argv = _CONVERT_ARGV if argv is None else argv

    digest = hashlib.sha256()
    digest.update(_FINGERPRINT_SCHEMA)

    digest.update(b"\0osm\0")
    with osm_path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)

    digest.update(b"\0src\0")
    for path in sorted(src_root.rglob("*")):
        if path.suffix not in _FINGERPRINT_SUFFIXES:
            continue
        if "__pycache__" in path.parts:
            continue
        if not path.is_file():
            continue
        digest.update(path.relative_to(src_root).as_posix().encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")

    digest.update(b"\0argv\0")
    digest.update("\0".join(argv).encode())

    return digest.hexdigest()


def _stamp_is_current(fingerprint: str) -> bool:
    """Return whether the on-disk stamp matches ``fingerprint``.

    A missing or unreadable stamp counts as stale, so an artifact produced
    before this mechanism existed is regenerated once rather than trusted.
    """
    try:
        return _STAMP_PATH.read_text(encoding="utf-8").strip() == fingerprint
    except OSError:
        return False


@pytest.fixture(scope="session", autouse=True)
def _ensure_nishishinjuku_xodr(tmp_path_factory: pytest.TempPathFactory) -> None:
    """Generate ``nishishinjuku_carla.xodr`` when it is missing or **stale**.

    The artifact is derived from the source OSM, the converter source tree and
    the Hydra configuration under it. It is regenerated whenever any of those
    change, keyed on :func:`_input_fingerprint` and recorded in a sidecar
    ``.inputs`` stamp next to the XODR. Existence alone is not enough: an XODR
    built on an older checkout is not a valid input to tests written for a
    newer one, and the resulting failure looks like a converter bug rather
    than a cache miss.

    Safe under ``pytest-xdist``: a cross-worker :class:`filelock.FileLock`
    serialises the staleness check and the convert subprocess, and the
    output is staged through a temp file then atomically renamed via
    :func:`os.replace`. This closes the TOCTOU race that let one worker
    read a partially-written XODR while another was still mid-write
    (issue #462).
    """
    if not _OSM_PATH.exists():
        # Checked before the fingerprint, which reads this file.
        pytest.skip(f"Source OSM not found: {_OSM_PATH}")

    fingerprint = _input_fingerprint()
    if _XODR_PATH.exists() and _stamp_is_current(fingerprint):
        return

    # ``getbasetemp().parent`` is the one directory that is shared across
    # every xdist worker for the same pytest invocation, so it is the
    # natural location for an inter-worker lock sentinel.
    lock_path = tmp_path_factory.getbasetemp().parent / "nishishinjuku_carla.xodr.lock"

    with FileLock(str(lock_path)):
        # Re-check inside the critical section: another worker may have
        # produced the file while we were blocked on the lock.
        if _XODR_PATH.exists() and _stamp_is_current(fingerprint):
            return

        # Invalidate before regenerating. If the conversion fails partway, the
        # next run must not find a stamp vouching for an artifact that was
        # never rebuilt.
        _STAMP_PATH.unlink(missing_ok=True)

        # Stage to a worker-unique temp path, then atomically rename so
        # other workers never observe a partially-written file.
        tmp_out = _XODR_PATH.with_name(f"{_XODR_PATH.name}.tmp.{os.getpid()}")
        try:
            subprocess.run(
                [
                    *_CONVERT_ARGV,
                    f"input_map_path={_OSM_PATH}",
                    f"output_map_path={tmp_out}",
                ],
                cwd=_PROJECT_ROOT,
                check=True,
            )
            os.replace(tmp_out, _XODR_PATH)
            # Stamp last: the window between the XODR landing and the stamp
            # appearing is a false *miss* (one extra reconversion), never a
            # false hit. The reverse order would produce a stamp vouching for
            # a file that is not there yet.
            _STAMP_PATH.write_text(fingerprint, encoding="utf-8")
        finally:
            if tmp_out.exists():
                try:
                    tmp_out.unlink()
                except OSError:
                    pass
