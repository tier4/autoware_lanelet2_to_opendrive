#!/usr/bin/env python3
"""Shrink a Lanelet2 map to a small one on which a check still fails the same way.

Given a map on which some check fails, and a command that says whether a
candidate map still shows that failure, this removes lanelets for as long as
the failure survives (delta debugging, Zeller & Hildebrandt 2002). The result
is a map to write a red test against: it converts in seconds rather than the
minutes a full map takes.

The command is the whole definition of "the same failure". It receives the
candidate map's path in place of ``{map}`` and exits 0 when the candidate is
still interesting. **Make it specific**: a command that only asks "does
something fail?" lets the search drift to a different defect that happens to
survive on a smaller map. Check for the same violation, on the same kind of
road, with the same message -- and make a candidate on which the converter
itself fails *not* interesting, unless the crash is the defect.

Only whole lanelets are removed. Every kept lanelet keeps its surveyed
geometry exactly; the closure of ways, nodes and regulatory elements is taken
by ``crop_lanelet2_map.write_excerpt``, so each candidate loads on its own.

**Preprocessing.** A map config's ``merge_operations`` /
``move_point_operations`` / ... name lanelet and point ids, and the converter
fails when a candidate no longer contains them (``nishishinjuku``'s
``Failed to move point 10030`` is the first to go). Convert candidates with
them switched off -- ``map.move_point_operations=null`` and so on, as
``conf/map/junction_cluster.yaml`` does for its fixture -- and first check,
on the full map, that the failure survives without them.

Usage::

    python reduce_lanelet2_map.py SOURCE.osm OUT.osm \\
        [--x X --y Y --w W --hops N] --test CMD... {map}

``--test`` takes the rest of the line, so it comes last. ``--x/--y/--w/--hops``
first crop SOURCE around the failure, exactly as ``crop_lanelet2_map.py``
does, so the search starts from tens of lanelets rather than a thousand. OUT
gets a sibling ``OUT.provenance.json`` recording the source, the crop, the
command and the kept ids.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, FrozenSet, Iterable, List, Optional, Sequence

import lxml.etree as ET

from crop_lanelet2_map import write_excerpt

Predicate = Callable[[FrozenSet[int]], bool]

MAP_PLACEHOLDER = "{map}"


def _as_text(captured: object) -> str:
    """Subprocess output as text, whichever form the exception carried it in."""
    if captured is None:
        return ""
    if isinstance(captured, bytes):
        return captured.decode("utf-8", errors="replace")
    return str(captured)


class NotInteresting(Exception):
    """The starting set does not show the failure, so there is nothing to reduce."""


def ddmin(items: Iterable[int], interesting: Predicate) -> List[int]:
    """Return a 1-minimal subset of *items* on which *interesting* holds.

    1-minimal: removing any single element of the result makes it
    uninteresting. Each distinct subset is evaluated at most once, and the
    empty set never. The result is sorted, and the search order is fixed by
    sorting the input, so equal inputs give equal results.
    """
    current = sorted(set(items))
    cache: Dict[FrozenSet[int], bool] = {}

    def test(subset: Sequence[int]) -> bool:
        key = frozenset(subset)
        if not key:
            return False
        if key not in cache:
            cache[key] = interesting(key)
        return cache[key]

    if not test(current):
        raise NotInteresting("the starting set does not show the failure")

    granularity = 2
    while len(current) >= 2:
        chunks = _split(current, granularity)
        reduced = False
        for chunk in chunks:
            if test(chunk):
                current, granularity, reduced = chunk, 2, True
                break
        if not reduced:
            for i in range(len(chunks)):
                complement = [x for j, c in enumerate(chunks) if j != i for x in c]
                if test(complement):
                    current = complement
                    granularity = max(granularity - 1, 2)
                    reduced = True
                    break
        if not reduced:
            if granularity >= len(current):
                break
            granularity = min(granularity * 2, len(current))
    return current


def _split(items: List[int], n: int) -> List[List[int]]:
    """Split *items* into *n* contiguous chunks whose sizes differ by at most one."""
    size, extra = divmod(len(items), n)
    chunks, start = [], 0
    for i in range(n):
        end = start + size + (1 if i < extra else 0)
        chunks.append(items[start:end])
        start = end
    return [c for c in chunks if c]


class MapPredicate:
    """Write the candidate map for a set of lanelet ids and run the user's command on it."""

    def __init__(
        self,
        source: Path,
        command: Sequence[str],
        work_dir: Path,
        timeout: Optional[float] = None,
        log: Callable[[str], None] = lambda _message: None,
    ) -> None:
        # Substring, not list membership: the Hydra form this tool's own
        # docstring recommends puts the placeholder inside a token --
        # `input_map_path={map}` -- and `MAP_PLACEHOLDER not in command`
        # on a list rejected exactly that, with no way around it short
        # of `bash -c`.
        if not any(MAP_PLACEHOLDER in argument for argument in command):
            raise ValueError(f"the test command must contain {MAP_PLACEHOLDER}")
        self.source = Path(source)
        self.command = list(command)
        self.work_dir = Path(work_dir)
        self.work_dir.mkdir(parents=True, exist_ok=True)
        self.timeout = timeout
        self.log = log
        self.evaluations = 0

    def __call__(self, keep: FrozenSet[int]) -> bool:
        self.evaluations += 1

        # One directory per evaluation. Sharing `candidate.osm` meant the
        # converter's own outputs -- it writes `candidate.xodr` and its
        # sidecars beside the input when given no output path -- survived
        # into the next evaluation, so a command that reads them could
        # judge candidate N on candidate N-1's output. That is the stale
        # artifact problem this tool exists to shrink, one directory over.
        attempt = self.work_dir / f"eval-{self.evaluations:04d}"
        attempt.mkdir(parents=True, exist_ok=True)
        candidate = attempt / "candidate.osm"

        census = write_excerpt(self.source, candidate, set(keep))

        argv = [a.replace(MAP_PLACEHOLDER, str(candidate)) for a in self.command]
        started = time.monotonic()
        stdout = stderr = ""
        try:
            result = subprocess.run(
                argv, capture_output=True, text=True, timeout=self.timeout
            )
            interesting = result.returncode == 0
            stdout, stderr = result.stdout, result.stderr
            verdict = f"exit {result.returncode}"
        except subprocess.TimeoutExpired as expired:
            interesting = False
            # `TimeoutExpired` types its captured output as `bytes | str |
            # None` regardless of `text=True`, so decode defensively rather
            # than assuming the str branch.
            stdout = _as_text(expired.stdout)
            stderr = _as_text(expired.stderr)
            verdict = f"timeout after {self.timeout} s"

        # Kept, not discarded: after a 40-evaluation run there is otherwise
        # no way to answer "which candidates failed, and why", which is what
        # makes a wrong reduction undiagnosable rather than merely possible.
        (attempt / "stdout.log").write_text(stdout, encoding="utf-8")
        (attempt / "stderr.log").write_text(stderr, encoding="utf-8")
        (attempt / "argv.txt").write_text(os.linesep.join(argv), encoding="utf-8")
        (attempt / "kept.txt").write_text(
            os.linesep.join(str(i) for i in sorted(keep)), encoding="utf-8"
        )

        # An excerpt is not simply a subset of its source: a regulatory
        # element whose other lanelet was cut away is dropped, and the
        # reference to it is removed from the lanelets that are kept (see
        # `crop_lanelet2_map._fully_inside`). A non-zero count here means
        # the candidate differs from the source in more than which lanelets
        # it has, so the behaviour being measured may have changed for a
        # reason the cut introduced. `write_excerpt` reports it and this
        # used to throw it away.
        dropped = int(census.get("regulatory_elements_dropped", 0) or 0)
        note = f", {dropped} regulatory element(s) dropped" if dropped else ""

        self.log(
            f"#{self.evaluations}: {len(keep)} lanelets -> "
            f"{'interesting' if interesting else 'not interesting'} "
            f"({verdict}, {time.monotonic() - started:.1f} s{note}) "
            f"[{attempt.name}]"
        )
        return interesting


@dataclass
class Reduction:
    kept: List[int]
    start_size: int
    evaluations: int
    seconds: float
    provenance: Dict[str, object] = field(default_factory=dict)


def lanelet_ids(osm: Path) -> List[int]:
    """Ids of the ``type=lanelet`` relations in *osm*."""
    root = ET.parse(str(osm)).getroot()
    return sorted(
        int(relation.get("id"))
        for relation in root.iter("relation")
        if any(
            tag.get("k") == "type" and tag.get("v") == "lanelet"
            for tag in relation.iter("tag")
        )
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reduce_map(
    source: Path,
    destination: Path,
    command: Sequence[str],
    work_dir: Path,
    start: Optional[Iterable[int]] = None,
    crop: Optional[Dict[str, float]] = None,
    timeout: Optional[float] = None,
    log: Callable[[str], None] = lambda _message: None,
) -> Reduction:
    """Reduce *source* to *destination* and write its provenance beside it.

    *start* is the set of lanelet ids to begin from (default: every lanelet in
    *source*). The search runs on an excerpt holding only those lanelets, so
    each candidate is written from a small file.
    """
    source, destination, work_dir = Path(source), Path(destination), Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    start_ids = sorted(set(start)) if start is not None else lanelet_ids(source)

    base = work_dir / "start.osm"
    write_excerpt(source, base, set(start_ids))
    predicate = MapPredicate(base, command, work_dir, timeout=timeout, log=log)

    started = time.monotonic()
    kept = ddmin(start_ids, predicate)
    seconds = time.monotonic() - started

    write_excerpt(base, destination, set(kept))

    # Re-ask the question about the file that is actually delivered.
    #
    # ddmin takes the first interesting chunk and never backtracks, and
    # every verdict is memoised, so a single false positive -- a candidate
    # the command called interesting for the wrong reason -- is permanent:
    # the search descends from it and the result is written with a
    # confident provenance file beside it. Nothing distinguished that from
    # a real reduction, because the predicate was never run on the result.
    confirmed = predicate(frozenset(kept))
    if not confirmed:
        raise NotInteresting(
            f"the reduced map at {destination} does not reproduce the "
            "failure. ddmin memoises verdicts and never backtracks, so "
            "this means some candidate was judged interesting for the "
            "wrong reason -- a flaky command, or one that exits 0 on a map "
            "the converter could not even load. The per-evaluation logs "
            f"under {work_dir} record each verdict and its output."
        )

    provenance: Dict[str, object] = {
        "tool": "reduce_lanelet2_map.py",
        "source": str(source),
        "source_sha256": _sha256(source),
        "crop": crop,
        "test_command": list(command),
        # The ids, not their count: ddmin's result depends on the starting
        # set and the order it is searched in (`sorted(set(start))`), so
        # the count alone does not let anyone re-derive this. Recovering
        # them from `crop` would mean re-running `select_lanelets`, whose
        # answer depends on the lanelet2 version and a hard-coded survey
        # origin.
        "start_lanelets": start_ids,
        "kept_lanelets": kept,
        "evaluations": predicate.evaluations,
        "seconds": round(seconds, 1),
        "result_reproduces_the_failure": confirmed,
    }
    destination.with_suffix(".provenance.json").write_text(
        json.dumps(provenance, indent=2) + "\n", encoding="utf-8"
    )
    return Reduction(kept, len(start_ids), predicate.evaluations, seconds, provenance)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("source", type=Path, help="Lanelet2 .osm the failure shows on")
    parser.add_argument("destination", type=Path, help="reduced .osm to write")
    parser.add_argument(
        "--test",
        nargs=argparse.REMAINDER,
        required=True,
        help=f"command; exits 0 while still interesting; {MAP_PLACEHOLDER} = candidate",
    )
    parser.add_argument("--x", type=float, help="crop: box south-west easting (m)")
    parser.add_argument("--y", type=float, help="crop: box south-west northing (m)")
    parser.add_argument("--w", type=float, help="crop: box side length (m)")
    parser.add_argument("--hops", type=int, default=1, help="crop: neighbour rounds")
    parser.add_argument("--timeout", type=float, help="seconds per evaluation")
    parser.add_argument(
        "--work-dir", type=Path, help="scratch directory (default: next to OUT)"
    )
    args = parser.parse_args(argv)

    start = None
    crop = None
    if any(getattr(args, name) is not None for name in ("x", "y", "w")):
        # All three or none. Guarding on `--x` alone sent `None` into the
        # crop box, where `x <= point[0] < x + None` raises TypeError after
        # the whole map has been loaded. `crop_lanelet2_map.py` validates
        # the group properly; this did not reuse that check.
        missing = [n for n in ("x", "y", "w") if getattr(args, n) is None]
        if missing:
            parser.error(
                "cropping needs --x, --y and --w together; missing "
                + ", ".join("--" + n for n in missing)
            )
        from crop_lanelet2_map import _load, select_lanelets

        lanelet_map, graph = _load(args.source)
        start = select_lanelets(lanelet_map, graph, args.x, args.y, args.w, args.hops)
        crop = {"x": args.x, "y": args.y, "w": args.w, "hops": args.hops}

    work_dir = args.work_dir or args.destination.with_suffix(".reduce-work")
    try:
        result = reduce_map(
            args.source,
            args.destination,
            args.test,
            work_dir,
            start=start,
            crop=crop,
            timeout=args.timeout,
            log=lambda message: print(message, file=sys.stderr, flush=True),
        )
    except NotInteresting as unusable:
        # The most likely first-use outcome -- the exit-code polarity the
        # wrong way round -- and the exception's own message says what to
        # check. Printing it beats a traceback that buries it.
        print(f"reduce_lanelet2_map: {unusable}", file=sys.stderr)
        return 2
    print(
        f"{result.start_size} -> {len(result.kept)} lanelets in "
        f"{result.evaluations} evaluations, {result.seconds:.0f} s: {result.kept}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
