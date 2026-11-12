#!/usr/bin/env python3
"""Report a map-specific conversion defect without publishing the map.

A defect that only shows on a map you may not share is still reportable. This
tool turns "it fails on my map" into something a maintainer can act on, in
three levels of disclosure -- each one a deliberate step, never a default:

* **L0 -- fingerprint.** Convert the map, run every instrument, and write
  what was found as *codes and counts*: no element id, no coordinate, no
  path. ``fingerprint`` writes it. It cannot be used to rebuild any part of
  the map, but it is still measurement data about someone's map: publishing
  it needs the owner's agreement once, not a judgement call per report.
* **L1 -- synthetic reproducer.** Shrink the map to the few lanelets on which
  the same finding still shows (``reduce``), then renumber, relocate, rotate
  and perturb them (``sanitize``) and confirm the finding survived. The
  *layout* of those few lanelets survives too -- it has to, or the defect
  would not -- so whether to publish it is the map owner's decision.
* **L2 -- the reduced excerpt itself** (``private/reduced.osm``). Real ids,
  real coordinates. Never attach it to a public issue.

Everything is written under one directory with two halves::

    <out>/public/    fingerprint.json, repro.osm, repro.json, issue.md
    <out>/private/   locations.json, reduced.osm, reduction logs, conversions

``leak-check`` verifies that nothing under ``public/`` carries a source id, a
source coordinate or a source path. ``sanitize`` and ``draft`` run it and
refuse to finish when it fails.

Usage (inside the project container; ``--`` separates Hydra overrides)::

    python scripts/map_issue.py fingerprint MAP.osm --out OUT -- map=<your config>
    python scripts/map_issue.py fingerprint MAP.osm --out OUT2 -- map=junction_cluster
    python scripts/map_issue.py reduce MAP.osm --from OUT2 --finding invariant:<code> \\
        -- map=junction_cluster
    python scripts/map_issue.py sanitize --from OUT2 --finding invariant:<code> \\
        -- map=junction_cluster
    python scripts/map_issue.py draft --from OUT2

``still-fails`` is the predicate ``reduce`` hands to the delta debugger; it is
exposed so the same question can be asked of any map by hand.

Procedure, and what each step is allowed to conclude:
``.claude/skills/map-issue-report/SKILL.md``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import os
import random
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

REPO = Path(__file__).resolve().parents[1]
TOOLS = REPO / "autoware_lanelet2_to_opendrive" / "test" / "data" / "tools"

INSTRUMENTS = ("crash", "invariant", "qc", "route")

#: Files ``public/`` may contain. Anything else there fails the leak check:
#: the directory is what gets attached, so an unexpected file is a leak until
#: somebody has looked at it.
PUBLIC_FILES = frozenset({"fingerprint.json", "repro.osm", "repro.json", "issue.md"})

#: Where a reproducer is placed unless told otherwise. It lies in MGRS square
#: 54SUE, the one ``conf/map/junction_cluster.yaml`` names, so the neutral
#: config converts a reproducer without further setup. The leak check rejects
#: a placement inside the source map's own bounding box; pass
#: ``--to-lat/--to-lon`` (and a config whose origin matches) in that case.
DEFAULT_PLACEMENT = (35.4, 139.4)

#: Perturbation levels tried in order, strongest first: (name, warp m, jitter
#: m). A defect that lives in centimetre-scale geometry can be erased by the
#: jitter, so ``sanitize`` steps down until the finding survives and records
#: which level that was. ``rigid`` moves and rotates but keeps every relative
#: distance exactly.
PERTURBATION_LADDER = (
    ("perturbed", 0.25, 0.02),
    ("warped", 0.25, 0.0),
    ("rigid", 0.0, 0.0),
)
WARP_WAVELENGTH_M = 150.0

#: How many scrubbed example messages a finding carries into the fingerprint.
MAX_EXAMPLES = 3

#: An absolute path: at least two segments, or one segment with an extension.
#: Deliberately not "anything after a slash" -- ``road 412/successor`` is a
#: location inside a document, and swallowing it would erase the part of a
#: message that says what kind of defect it is.
_PATH = re.compile(
    r"(?<![\w>])(?:[A-Za-z]:[\\/]|/)" r"(?:[\w.~-]+(?:[\\/][\w.~-]*)+|[\w~-]+\.\w{2,5})"
)
_NUMBER = re.compile(r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")
_ROAD = re.compile(r"[Rr]oad (\d+)")
_EXCEPTION = re.compile(
    r"^((?:[A-Za-z_][\w.]*\.)?[A-Z]\w*(?:Error|Exception|Exit|Interrupt))(?::\s*(.*))?$"
)


def scrub(text: str) -> str:
    """Return *text* with every path and every number replaced by a placeholder.

    A message such as ``road 412 meets road 87 at its start 0.071 m away``
    names two ids and a measurement of the source map. Scrubbed, it keeps the
    part that identifies the *kind* of defect -- ``road <n> meets road <n> at
    its start <n> m away`` -- which is all a fingerprint may carry.
    """
    return _NUMBER.sub("<n>", _PATH.sub("<path>", text))


@dataclass
class Finding:
    """One kind of defect one instrument reported.

    ``instrument``, ``code``, ``count`` and ``examples`` are public.
    ``roads`` and ``lanelets`` say *where*, in the source map's own ids, and
    are written only to ``private/locations.json``.
    """

    instrument: str
    code: str
    count: int = 0
    examples: List[str] = field(default_factory=list)
    roads: List[List[int]] = field(default_factory=list)
    lanelets: List[List[int]] = field(default_factory=list)

    @property
    def key(self) -> str:
        return f"{self.instrument}:{self.code}"

    def add(
        self, message: str, roads: Iterable[int] = (), lanelets: Iterable[int] = ()
    ) -> None:
        self.count += 1
        example = scrub(message)
        if example not in self.examples and len(self.examples) < MAX_EXAMPLES:
            self.examples.append(example)
        self.roads.append(sorted(set(roads)))
        self.lanelets.append(sorted(set(lanelets)))

    def public(self) -> Dict[str, object]:
        return {
            "finding": self.key,
            "count": self.count,
            "examples": list(self.examples),
        }


class Findings:
    """Findings keyed by ``instrument:code``, plus what could not be measured."""

    def __init__(self) -> None:
        self.by_key: Dict[str, Finding] = {}
        #: instrument -> scrubbed reason it did not run to completion. An
        #: instrument that stopped early reports only what it saw before it
        #: stopped, so its counts are a lower bound and the fingerprint has
        #: to say so.
        self.incomplete: Dict[str, str] = {}
        self.notes: Dict[str, object] = {}

    def get(self, instrument: str, code: str) -> Finding:
        key = f"{instrument}:{code}"
        if key not in self.by_key:
            self.by_key[key] = Finding(instrument, code)
        return self.by_key[key]

    def sorted(self) -> List[Finding]:
        return [self.by_key[k] for k in sorted(self.by_key)]


# --------------------------------------------------------------------------
# Conversion


@dataclass
class Conversion:
    """One run of the converter and where its outputs are."""

    map_path: Path
    out_dir: Path
    returncode: int
    seconds: float
    log: str

    @property
    def xodr(self) -> Path:
        return self.out_dir / "out.xodr"

    @property
    def mapping(self) -> Path:
        return self.out_dir / "out.mapping.json"

    @property
    def preprocessed(self) -> Optional[Path]:
        """The preprocessed map the converter saved beside the output, if any.

        Preprocessing rewrites lanelets -- a merge replaces two ids with a
        new one -- so every id in the mapping sidecar refers to *this* file,
        not to the input. It exists only when preprocessing actually ran.
        """
        found = sorted(self.out_dir.glob("out*preprocessed.osm"))
        return found[0] if found else None

    @property
    def complete(self) -> bool:
        """True when the converter exited 0 and left a closed document.

        The exit code alone is not enough: a run killed while writing leaves
        a truncated file behind, and an instrument pointed at it measures the
        truncation.
        """
        if self.returncode != 0 or not self.xodr.is_file():
            return False
        with open(self.xodr, "rb") as handle:
            handle.seek(max(0, self.xodr.stat().st_size - 256))
            return b"</OpenDRIVE>" in handle.read()


def run_conversion(
    map_path: Path,
    out_dir: Path,
    overrides: Sequence[str],
    timeout: Optional[float] = None,
) -> Conversion:
    """Convert *map_path* into *out_dir* and keep the log."""
    out_dir.mkdir(parents=True, exist_ok=True)
    for stale in out_dir.glob("out*"):
        if stale.is_file():
            stale.unlink()
    command = [
        sys.executable,
        "-m",
        "autoware_lanelet2_to_opendrive.main",
        f"input_map_path={map_path}",
        f"output_map_path={out_dir / 'out.xodr'}",
        f"hydra.run.dir={out_dir / 'hydra'}",
        *overrides,
    ]
    started = time.monotonic()
    try:
        done = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout, cwd=str(REPO)
        )
        code, log = done.returncode, done.stdout + done.stderr
    except subprocess.TimeoutExpired as expired:
        code = -1
        log = f"TimeoutExpired: conversion exceeded {expired.timeout} s"
    seconds = time.monotonic() - started
    (out_dir / "convert.log").write_text(log, encoding="utf-8")
    return Conversion(Path(map_path), out_dir, code, seconds, log)


def crash_of(log: str) -> Optional[Tuple[str, str]]:
    """Return ``(exception type, message)`` from the last traceback in *log*."""
    lines = log.splitlines()
    for index in range(len(lines) - 1, -1, -1):
        match = _EXCEPTION.match(lines[index].strip())
        if match:
            return match.group(1), (match.group(2) or "").strip()
    return None


# --------------------------------------------------------------------------
# Instruments


def _measure_crash(conversion: Conversion, findings: Findings) -> None:
    if conversion.complete:
        return
    crash = crash_of(conversion.log) or (
        "NoTraceback",
        f"exit code {conversion.returncode}",
    )
    kind, message = crash
    # The message is part of the identity: two ValueErrors are two defects.
    # Scrubbed first, so the same defect at another place hashes the same.
    digest = hashlib.sha1(scrub(message).encode("utf-8")).hexdigest()[:8]
    findings.get("crash", f"{kind}:{digest}").add(message)


def _measure_invariants(conversion: Conversion, findings: Findings) -> None:
    from autoware_lanelet2_to_opendrive.xodr_invariants import check_invariants

    for violation in check_invariants(conversion.xodr):
        roads = [int(n) for n in _ROAD.findall(f"{violation.where} {violation.detail}")]
        findings.get("invariant", violation.code).add(
            f"{violation.where}: {violation.detail}", roads=roads
        )


def _measure_qc(conversion: Conversion, findings: Findings) -> None:
    import lxml.etree as LET
    from qc_baselib import Configuration, IssueSeverity, Result
    from qc_baselib.models.result import StatusType
    from qc_opendrive import constants
    from qc_opendrive.main import run_checks

    from autoware_lanelet2_to_opendrive.qc_validate import load_ignore_patterns

    # The bundle is driven here rather than through ``qc_validate`` so the
    # populated Result is in hand: the gate returns one number, and this
    # needs the per-checker status and each issue's rule and location.
    config = Configuration()
    config.set_config_param("InputFile", str(conversion.xodr))
    config.register_checker_bundle(constants.BUNDLE_NAME)
    config.set_checker_bundle_param(
        constants.BUNDLE_NAME, "resultFile", str(conversion.out_dir / "qc.xqar")
    )
    result = Result()
    result.register_checker_bundle(
        name=constants.BUNDLE_NAME,
        description="OpenDrive checker bundle",
        version=constants.BUNDLE_VERSION,
        summary="",
    )
    result.set_result_version(version=constants.BUNDLE_VERSION)
    run_checks(config, result)
    document = LET.parse(str(conversion.xodr))
    ignore = [re.compile(p) for p in load_ignore_patterns()]
    tally: Counter = Counter()
    ignored = 0
    checker_ids = result.get_checker_ids(constants.BUNDLE_NAME)
    for checker_id in checker_ids:
        checker = result.get_checker_result(constants.BUNDLE_NAME, checker_id)
        tally[checker.status.name.lower()] += 1
        if checker.status == StatusType.SKIPPED:
            continue
        for issue in checker.issues:
            if issue.level != IssueSeverity.ERROR:
                continue
            texts = [issue.description] + [
                loc.description for loc in issue.locations if loc.description
            ]
            if any(pattern.search(text) for pattern in ignore for text in texts):
                ignored += 1
                continue
            roads: Set[int] = set()
            for location in issue.locations:
                for xml_location in location.xml_location or []:
                    roads.update(_roads_at(document, xml_location.xpath))
            findings.get("qc", issue.rule_uid or checker_id).add(
                issue.description, roads=roads
            )
    # Without this line an error count means nothing: a checker that was
    # skipped or that stopped on an exception contributes zero issues, and
    # zero issues reads as "clean".
    findings.notes["qc_checkers"] = {
        "total": len(checker_ids),
        "completed": tally.get("completed", 0),
        "skipped": tally.get("skipped", 0),
        "errored": tally.get("error", 0),
        "errors_matching_the_ignore_list": ignored,
    }


def _roads_at(document: object, xpath: str) -> Set[int]:
    """Ids of the ``<road>`` elements an issue's xpath points into."""
    found: Set[int] = set()
    try:
        nodes = document.xpath(xpath)  # type: ignore[attr-defined]
    except Exception:
        return found
    for node in nodes if isinstance(nodes, list) else []:
        while node is not None and getattr(node, "tag", None) != "road":
            node = node.getparent()
        if node is not None and (node.get("id") or "").lstrip("-").isdigit():
            found.add(int(node.get("id")))
    return found


def _measure_route(conversion: Conversion, findings: Findings) -> None:
    from autoware_lanelet2_to_opendrive.route_fidelity import measure

    source = conversion.preprocessed or conversion.map_path
    result = measure(source, conversion.xodr, conversion.mapping)
    for succession in result.missing:
        findings.get("route", "missing_succession").add(
            "a succession the map states is not traversable in the output",
            lanelets=[succession.source_lanelet, succession.target_lanelet],
        )
    for (road_a, _lane_a), (road_b, _lane_b) in result.surplus:
        findings.get("route", "surplus_transition").add(
            "the output allows a transition the map does not state",
            roads=[road_a, road_b],
        )
    for link in result.dangling:
        findings.get("route", "dangling_lane_link").add(
            f"a lane link at a road's {link.end} names a lane that does not exist",
            roads=[link.road, link.target_road],
        )
    findings.notes["route"] = {
        "stated": result.stated,
        "internal": result.internal,
        "unmapped": result.unmapped,
        "emitted": result.emitted,
    }


_MEASURES = {
    "invariant": _measure_invariants,
    "qc": _measure_qc,
    "route": _measure_route,
}


def measure_findings(
    conversion: Conversion, instruments: Iterable[str] = INSTRUMENTS
) -> Findings:
    """Run *instruments* over *conversion* and collect what they report."""
    findings = Findings()
    wanted = set(instruments)
    _measure_crash(conversion, findings)
    if not conversion.complete:
        for name in sorted(wanted - {"crash"}):
            findings.incomplete[name] = "the conversion did not complete"
        return findings
    for name in sorted(wanted - {"crash"}):
        try:
            _MEASURES[name](conversion, findings)
        except Exception as failure:  # an instrument failing is a datum
            findings.incomplete[name] = scrub(f"{type(failure).__name__}: {failure}")
    return findings


def still_fails(findings: Findings, finding: str, match: Optional[str] = None) -> bool:
    """True when *findings* still shows *finding* (and an example matches *match*).

    "Something fails" is not the question. A reduction that only asks that
    drifts onto whichever defect survives longest on a small map; the answer
    has to be the same code from the same instrument.
    """
    found = findings.by_key.get(finding)
    if found is None or found.count == 0:
        return False
    if match is None:
        return True
    pattern = re.compile(match)
    return any(pattern.search(example) for example in found.examples)


# --------------------------------------------------------------------------
# The map: census, and where a finding is


def _parse_osm(path: Path) -> ET.Element:
    with open(path, "rb") as handle:
        head = handle.read(4096)
    if b"<!DOCTYPE" in head or b"<!ENTITY" in head:
        raise ValueError(f"{path} carries a DOCTYPE or ENTITY declaration")
    return ET.parse(str(path)).getroot()


def _tags(element: ET.Element) -> Dict[str, str]:
    return {t.get("k", ""): t.get("v", "") for t in element.findall("tag")}


def census(path: Path) -> Dict[str, object]:
    """Counts describing *path*, none of which locate or identify it."""
    root = _parse_osm(path)
    lanelets: Counter = Counter()
    regulatory: Counter = Counter()
    node_tags: Counter = Counter()
    nodes = ways = 0
    for element in root:
        if element.tag == "node":
            nodes += 1
            node_tags.update(k for k in _tags(element) if k in _NODE_TAGS_OF_NOTE)
        elif element.tag == "way":
            ways += 1
        elif element.tag == "relation":
            tags = _tags(element)
            if tags.get("type") == "lanelet":
                lanelets[tags.get("subtype", "(none)")] += 1
            elif tags.get("type") == "regulatory_element":
                regulatory[tags.get("subtype", "(none)")] += 1
    return {
        "lanelets": sum(lanelets.values()),
        "lanelets_by_subtype": dict(sorted(lanelets.items())),
        "regulatory_elements_by_subtype": dict(sorted(regulatory.items())),
        "nodes": nodes,
        "ways": ways,
        "node_tags_present": sorted(node_tags),
    }


_NODE_TAGS_OF_NOTE = ("ele", "local_x", "local_y", "mgrs_code")


def lanelets_of_roads(mapping_path: Path) -> Dict[int, Set[int]]:
    """Invert the mapping sidecar: road id -> the lanelets it was built from.

    Every field that attaches a lanelet to a road is folded in. Reading only
    ``lanelet_to_road_and_lane`` under-counts: a chain-merged road, a
    duplicated fork stage and a lanelet emitted on several roads are each
    recorded in a field of their own.
    """
    with open(mapping_path, encoding="utf-8") as handle:
        data = json.load(handle)
    roads: Dict[int, Set[int]] = defaultdict(set)
    for lanelet, slot in (data.get("lanelet_to_road_and_lane") or {}).items():
        roads[int(slot[0])].add(int(lanelet))
    for lanelet, slots in (data.get("lanelet_additional_road_lanes") or {}).items():
        for slot in slots:
            roads[int(slot[0])].add(int(lanelet))
    for name in ("chain_merged_roads", "exit_fork_duplicate_roads"):
        for road, other in (data.get(name) or {}).items():
            # Both fields relate one road to another road (or roads) holding
            # the same lanelets; whatever either side has, both sides get.
            others = other if isinstance(other, list) else [other]
            group = {int(road)} | {
                int(o) for o in others if str(o).lstrip("-").isdigit()
            }
            merged = set().union(*(roads.get(r, set()) for r in group))
            for member in group:
                roads[member] |= merged
    return roads


def _road_neighbours(xodr: Path) -> Dict[int, Set[int]]:
    """Road id -> the roads its ``<link>`` and its junction's connections name."""
    neighbours: Dict[int, Set[int]] = defaultdict(set)
    root = ET.parse(str(xodr)).getroot()
    junction_roads: Dict[str, Set[int]] = defaultdict(set)
    for junction in root.iter("junction"):
        for connection in junction.iter("connection"):
            for name in ("incomingRoad", "connectingRoad"):
                value = connection.get(name) or ""
                if value.lstrip("-").isdigit():
                    junction_roads[junction.get("id", "")].add(int(value))
    for road in root.iter("road"):
        rid = int(road.get("id", "-1"))
        link = road.find("link")
        for end in link if link is not None else []:
            target = end.get("elementId") or ""
            if end.get("elementType") == "junction":
                neighbours[rid] |= junction_roads.get(target, set())
            elif target.lstrip("-").isdigit():
                neighbours[rid].add(int(target))
    for members in junction_roads.values():
        for member in members:
            neighbours[member] |= members
    return neighbours


def locate(conversion: Conversion, findings: Findings) -> Dict[str, List[List[int]]]:
    """For each finding, the lanelets each of its instances sits on.

    A road with no backing lanelet -- a synthetic connector -- is located by
    the roads it is linked to, since the lanelets that produced it are the
    ones on either side.
    """
    by_road = lanelets_of_roads(conversion.mapping)
    neighbours = _road_neighbours(conversion.xodr)
    located: Dict[str, List[List[int]]] = {}
    for finding in findings.sorted():
        instances: List[List[int]] = []
        for roads, lanelets in zip(finding.roads, finding.lanelets):
            here: Set[int] = set(lanelets)
            for road in roads:
                if by_road.get(road):
                    here |= by_road[road]
                else:
                    for other in neighbours.get(road, ()):
                        here |= by_road.get(other, set())
            if here:
                instances.append(sorted(here))
        located[finding.key] = instances
    return located


def expand(source: Path, lanelets: Iterable[int], hops: int) -> List[int]:
    """*lanelets* plus *hops* rounds of routing and lateral neighbours."""
    sys.path.insert(0, str(TOOLS))
    from crop_lanelet2_map import _load, _neighbours  # type: ignore[import-not-found]

    lanelet_map, graph = _load(source)
    layer = lanelet_map.laneletLayer
    keep = {i for i in lanelets if layer.exists(i)}
    frontier = set(keep)
    for _ in range(hops):
        reached: Set[int] = set()
        for lanelet_id in frontier:
            reached |= _neighbours(graph, layer.get(lanelet_id))
        frontier = reached - keep
        keep |= frontier
    return sorted(keep)


# --------------------------------------------------------------------------
# Leak check


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


def _nodes(root: ET.Element) -> List[ET.Element]:
    return [e for e in root if e.tag == "node"]


def _bbox(nodes: Sequence[ET.Element]) -> Tuple[float, float, float, float]:
    lats = [float(n.get("lat", "nan")) for n in nodes]
    lons = [float(n.get("lon", "nan")) for n in nodes]
    return min(lats), max(lats), min(lons), max(lons)


def _mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def leak_check(
    public: Path, source: Optional[Path], reduced: Optional[Path]
) -> List[Check]:
    """Verify nothing under *public* carries the source map's identity.

    Each check asks what a leak would look like and looks for it directly;
    none of them infers safety from how the files were produced. *source* is
    the full map and *reduced* the excerpt the reproducer was rebuilt from.
    """
    checks: List[Check] = []

    def add(name: str, ok: bool, detail: str) -> None:
        checks.append(Check(name, ok, detail))

    present = sorted(p.name for p in public.iterdir()) if public.is_dir() else []
    unexpected = sorted(set(present) - PUBLIC_FILES)
    add(
        "only expected files",
        not unexpected,
        f"unexpected: {unexpected}" if unexpected else f"{present}",
    )

    source_nodes = _nodes(_parse_osm(source)) if source else []
    reduced_root = _parse_osm(reduced) if reduced else None
    repro = public / "repro.osm"

    if repro.is_file():
        root = _parse_osm(repro)
        nodes = _nodes(root)

        ids = sorted(int(e.get("id", "0")) for e in root)
        fresh = ids == list(range(1, len(ids) + 1))
        add(
            "ids are a fresh counter",
            fresh,
            f"{len(ids)} elements numbered 1..{len(ids)}"
            if fresh
            else "ids are not 1..N, so they were not all renumbered",
        )

        bearing = Counter(
            key
            for element in root
            for key in _tags(element)
            if key == "mgrs_code" or key.endswith("_id") or key == "intersection_area"
        )
        add(
            "no tag carries a source id or position",
            not bearing,
            f"found {dict(bearing)}" if bearing else "none",
        )

        elevations = [float(t["ele"]) for t in map(_tags, nodes) if "ele" in t]
        add(
            "elevation is re-origined",
            abs(_mean(elevations)) < 0.1,
            f"mean ele {_mean(elevations):+.3f} m over {len(elevations)} nodes",
        )
        local = [
            (float(t["local_x"]), float(t["local_y"]))
            for t in map(_tags, nodes)
            if "local_x" in t and "local_y" in t
        ]
        centred = (
            abs(_mean([p[0] for p in local])) < 1.0
            and abs(_mean([p[1] for p in local])) < 1.0
        )
        add(
            "local_x/local_y are re-origined",
            centred,
            f"{len(local)} nodes, mean "
            f"({_mean([p[0] for p in local]):+.3f}, {_mean([p[1] for p in local]):+.3f})",
        )

        if reduced_root is not None:
            before = _nodes(reduced_root)
            same = sum(
                1
                for a, b in zip(before, nodes)
                if abs(float(a.get("lat", "nan")) - float(b.get("lat", "nan"))) < 1e-9
                or abs(float(a.get("lon", "nan")) - float(b.get("lon", "nan"))) < 1e-9
            )
            add(
                "no node keeps its latitude or longitude",
                len(before) == len(nodes) and same == 0,
                f"{same} of {len(nodes)} nodes unchanged"
                if len(before) == len(nodes)
                else f"node count differs: {len(before)} -> {len(nodes)}",
            )

        if source_nodes:
            s_lat0, s_lat1, s_lon0, s_lon1 = _bbox(source_nodes)
            r_lat0, r_lat1, r_lon0, r_lon1 = _bbox(nodes)
            apart = (
                r_lat1 < s_lat0 or r_lat0 > s_lat1 or r_lon1 < s_lon0 or r_lon0 > s_lon1
            )
            add(
                "placed outside the source map's bounding box",
                apart,
                "disjoint"
                if apart
                else "the reproducer overlaps the source map; choose another "
                "--to-lat/--to-lon",
            )

    # Text: the fingerprint, the reproducer's record and the issue draft.
    forbidden: Dict[str, str] = {}
    for label, path in (("source", source), ("reduced", reduced)):
        if path and path.is_file():
            forbidden[f"{label} file name"] = path.name
            forbidden[f"{label} sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
            forbidden[f"{label} directory"] = str(path.resolve().parent)
    source_ids: Set[str] = set()
    if reduced_root is not None:
        # Only ids long enough not to collide with a count by accident.
        source_ids = {
            e.get("id", "") for e in reduced_root if len(e.get("id", "")) >= 4
        }
    box = _bbox(source_nodes) if source_nodes else None
    for name in sorted(set(present) & {"fingerprint.json", "repro.json", "issue.md"}):
        text = (public / name).read_text(encoding="utf-8")
        hits = [what for what, needle in forbidden.items() if needle and needle in text]
        tokens = set(re.findall(r"(?<![\w.])\d{4,}(?!\w|\.\d)", text))
        id_hits = sorted(tokens & source_ids)
        coordinate_hits = []
        if box:
            for token in re.findall(r"\d+\.\d{4,}", text):
                value = float(token)
                if box[0] <= value <= box[1] or box[2] <= value <= box[3]:
                    coordinate_hits.append(token)
        add(
            f"{name}: no source path, hash, id or coordinate",
            not (hits or id_hits or coordinate_hits),
            "clean"
            if not (hits or id_hits or coordinate_hits)
            else f"paths/hashes {hits}; numbers equal to source ids {id_hits[:5]}; "
            f"numbers inside the source bounding box {coordinate_hits[:5]}",
        )
    return checks


def format_checks(checks: Sequence[Check]) -> str:
    lines = [f"  {'ok  ' if c.ok else 'LEAK'}  {c.name}: {c.detail}" for c in checks]
    verdict = "PASS" if all(c.ok for c in checks) else "FAIL"
    return "\n".join(lines + [f"leak-check: {verdict}"])


# --------------------------------------------------------------------------
# Subcommands


def _split_overrides(argv: Sequence[str]) -> Tuple[List[str], List[str]]:
    argv = list(argv)
    if "--" in argv:
        cut = argv.index("--")
        return argv[:cut], argv[cut + 1 :]
    return argv, []


def _public_overrides(overrides: Sequence[str]) -> List[str]:
    """Overrides as they may be published: keys kept, any path value withheld."""
    shown = []
    for override in overrides:
        key, _, value = override.partition("=")
        shown.append(f"{key}=<path>" if _PATH.fullmatch(value) else override)
    return shown


def _converter_identity() -> Dict[str, object]:
    identity: Dict[str, object] = {}
    try:
        from importlib.metadata import version

        identity["version"] = version("autoware-lanelet2-to-opendrive")
    except Exception:
        identity["version"] = "unknown"
    try:
        head = subprocess.run(
            ["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "-C", str(REPO), "status", "--porcelain", "--untracked-files=no"],
            capture_output=True,
            text=True,
        ).stdout.strip()
        identity["commit"] = head or "unknown"
        identity["uncommitted_changes"] = bool(dirty)
    except OSError:
        identity["commit"] = "unknown"
    return identity


def _write_json(path: Path, data: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _read_json(path: Path) -> Dict[str, object]:
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def fingerprint_document(
    map_path: Path,
    conversion: Conversion,
    findings: Findings,
    overrides: Sequence[str],
) -> Dict[str, object]:
    """The L0 report: what was found, with nothing that says where."""
    return {
        "schema": 1,
        "converter": _converter_identity(),
        "overrides": _public_overrides(overrides),
        "map_census": census(map_path),
        "conversion": {
            "completed": conversion.complete,
            "seconds": round(conversion.seconds, 1),
            "preprocessing_ran": conversion.preprocessed is not None,
        },
        "instruments_incomplete": dict(sorted(findings.incomplete.items())),
        "notes": findings.notes,
        "findings": [f.public() for f in findings.sorted()],
    }


def cmd_fingerprint(args: argparse.Namespace, overrides: List[str]) -> int:
    out = Path(args.out)
    conversion = run_conversion(
        Path(args.map), out / "private" / "conversion", overrides, args.timeout
    )
    findings = measure_findings(conversion)
    document = fingerprint_document(Path(args.map), conversion, findings, overrides)
    _write_json(out / "public" / "fingerprint.json", document)
    located = locate(conversion, findings) if conversion.complete else {}
    _write_json(
        out / "private" / "locations.json",
        {
            "source": str(Path(args.map).resolve()),
            "preprocessing_ran": conversion.preprocessed is not None,
            "lanelets_by_finding": located,
        },
    )
    print(
        f"conversion: {'completed' if conversion.complete else 'DID NOT COMPLETE'}"
        f" in {conversion.seconds:.0f} s"
    )
    for name, reason in sorted(findings.incomplete.items()):
        print(f"  instrument '{name}' incomplete: {reason}")
    if "qc_checkers" in findings.notes:
        print(f"  qc checkers: {findings.notes['qc_checkers']}")
    for finding in findings.sorted():
        places = len(located.get(finding.key, []))
        print(f"  {finding.key}  x{finding.count}  ({places} located)")
    if not findings.by_key:
        print("  no findings")
    print(f"wrote {out / 'public' / 'fingerprint.json'}")
    return 0


def cmd_still_fails(args: argparse.Namespace, overrides: List[str]) -> int:
    candidate = Path(args.map)
    instrument = args.finding.split(":", 1)[0]
    conversion = run_conversion(
        candidate, candidate.parent / "conversion", overrides, args.timeout
    )
    if instrument != "crash" and not conversion.complete:
        # A candidate the converter cannot convert is a *different* failure,
        # and calling it interesting is how a reduction ends on a broken map.
        print("not interesting: the conversion did not complete")
        return 1
    findings = measure_findings(conversion, [instrument])
    if instrument in findings.incomplete:
        print(f"not interesting: {findings.incomplete[instrument]}")
        return 1
    if still_fails(findings, args.finding, args.match):
        print(f"interesting: {args.finding} x{findings.by_key[args.finding].count}")
        return 0
    print(f"not interesting: {args.finding} is gone; saw {sorted(findings.by_key)}")
    return 1


def cmd_reduce(args: argparse.Namespace, overrides: List[str]) -> int:
    sys.path.insert(0, str(TOOLS))
    from reduce_lanelet2_map import (  # type: ignore[import-not-found]
        NotInteresting,
        reduce_map,
    )

    out = Path(getattr(args, "from"))
    source = Path(args.map)
    locations = _read_json(out / "private" / "locations.json")
    if locations.get("preprocessing_ran"):
        print(
            "reduce: the fingerprint in --from was taken with preprocessing on, so "
            "its lanelet ids are those of the preprocessed map and an excerpt of "
            "the input will not contain them. Take a fingerprint with a map "
            "config that names no operations (map=junction_cluster) and reduce "
            "from that.",
            file=sys.stderr,
        )
        return 2
    instances = locations.get("lanelets_by_finding", {}).get(args.finding)  # type: ignore[union-attr]
    command = [sys.executable, str(Path(__file__).resolve()), "still-fails"]
    command += ["--finding", args.finding]
    if args.match:
        command += ["--match", args.match]
    if args.timeout:
        command += ["--timeout", str(args.timeout)]
    command += ["{map}", "--", *overrides]

    def log(message: str) -> None:
        print(message, file=sys.stderr, flush=True)

    destination = out / "private" / "reduced.osm"
    if not instances:
        log(
            f"reduce: {args.finding} has no located instance (a crash has none); "
            "starting from the whole map. Every evaluation converts a large "
            "candidate until the search has narrowed."
        )
        starts: List[Optional[List[int]]] = [None]
    else:
        # The smallest instance, by size then by ids: the cheapest start, and
        # the same choice on every run.
        seed = min(instances, key=lambda ids: (len(ids), ids))
        starts = [expand(source, seed, hops) for hops in args.hops]
    for attempt, start in enumerate(starts):
        size = "all" if start is None else len(start)
        log(f"reduce: starting from {size} lanelets")
        try:
            result = reduce_map(
                source,
                destination,
                command,
                out / "private" / f"reduce-work-{attempt}",
                start=start,
                timeout=args.timeout,
                log=log,
            )
        except NotInteresting as unusable:
            log(f"reduce: {unusable}")
            continue
        print(
            f"{result.start_size} -> {len(result.kept)} lanelets in "
            f"{result.evaluations} evaluations, {result.seconds:.0f} s"
        )
        print(f"wrote {destination}  (PRIVATE: real ids and coordinates)")
        return 0
    print(
        "reduce: no starting neighbourhood shows the finding. Either it needs "
        "more of the map around it (raise --hops), or it does not survive "
        "without preprocessing -- check that the --from fingerprint lists it.",
        file=sys.stderr,
    )
    return 2


def cmd_sanitize(args: argparse.Namespace, overrides: List[str]) -> int:
    sys.path.insert(0, str(TOOLS))
    from rebuild_as_synthetic import rebuild  # type: ignore[import-not-found]

    out = Path(getattr(args, "from"))
    public, private = out / "public", out / "private"
    reduced = private / "reduced.osm"
    source = Path(_read_json(private / "locations.json")["source"])  # type: ignore[arg-type]
    public.mkdir(parents=True, exist_ok=True)
    seed = args.seed if args.seed is not None else int.from_bytes(os.urandom(4), "big")
    rotate = (
        args.rotate if args.rotate is not None else random.Random(seed).uniform(20, 340)
    )
    instrument = args.finding.split(":", 1)[0]
    repro = public / "repro.osm"

    for level, warp, jitter in PERTURBATION_LADDER:
        rebuild(
            reduced,
            repro,
            rotate,
            args.to_lat,
            args.to_lon,
            warp,
            jitter,
            WARP_WAVELENGTH_M,
            seed,
        )
        conversion = run_conversion(
            repro, private / f"sanitize-{level}", overrides, args.timeout
        )
        findings = measure_findings(conversion, [instrument])
        survived = (
            (instrument == "crash" or conversion.complete)
            and instrument not in findings.incomplete
            and still_fails(findings, args.finding, args.match)
        )
        print(
            f"sanitize: level '{level}' (warp {warp} m, jitter {jitter} m): "
            f"{'finding survives' if survived else 'finding is gone'}"
        )
        if not survived:
            continue
        everything = measure_findings(conversion)
        _write_json(
            public / "repro.json",
            {
                "schema": 1,
                "reproduces": args.finding,
                "sanitization": {
                    "level": level,
                    "warp_m": warp,
                    "warp_wavelength_m": WARP_WAVELENGTH_M,
                    "jitter_sigma_m": jitter,
                    "renumbered": True,
                    "relocated_and_rotated": True,
                },
                "map_census": census(repro),
                "converter": _converter_identity(),
                "command": "convert input_map_path=repro.osm "
                + " ".join(_public_overrides(overrides)),
                "conversion_seconds": round(conversion.seconds, 1),
                "instruments_incomplete": dict(sorted(everything.incomplete.items())),
                "notes": everything.notes,
                "findings": [f.public() for f in everything.sorted()],
            },
        )
        # The transform that would undo the relocation stays private.
        _write_json(
            private / "sanitize.json",
            {"seed": seed, "rotate_degrees": rotate, "level": level},
        )
        checks = leak_check(public, source, reduced)
        print(format_checks(checks))
        if not all(c.ok for c in checks):
            repro.unlink()
            (public / "repro.json").unlink()
            print(
                "sanitize: the reproducer was removed because the leak check failed",
                file=sys.stderr,
            )
            return 1
        print(f"wrote {repro}")
        return 0

    if repro.exists():
        repro.unlink()
    print(
        "sanitize: the finding does not survive even a rigid move, so it depends "
        "on something the rebuild changes (an id-bearing tag, the absolute "
        "position, the elevation origin). Report at L0, and say so.",
        file=sys.stderr,
    )
    return 2


def cmd_leak_check(args: argparse.Namespace, _overrides: List[str]) -> int:
    out = Path(getattr(args, "from"))
    private = out / "private"
    source = Path(_read_json(private / "locations.json")["source"])  # type: ignore[arg-type]
    reduced = private / "reduced.osm"
    checks = leak_check(out / "public", source, reduced if reduced.is_file() else None)
    print(format_checks(checks))
    return 0 if all(c.ok for c in checks) else 1


def draft_issue(
    fingerprint: Dict[str, object], repro: Optional[Dict[str, object]]
) -> str:
    """The issue body, from the public documents alone."""
    converter = fingerprint["converter"]  # type: ignore[assignment]
    conversion = fingerprint["conversion"]  # type: ignore[assignment]
    lines: List[str] = []
    target = repro["reproduces"] if repro else None
    lines += ["## 🐛 Finding", ""]
    if target:
        lines += [f"`{target}` -- reproduced on the attached `repro.osm`.", ""]
    else:
        lines += [
            "<!-- Name the finding this issue is about: one `instrument:code` "
            "from the table below. One issue per finding. -->",
            "",
        ]
    lines += ["## 📊 Fingerprint of the source map (L0: codes and counts only)", ""]
    lines += [
        f"- **Converter**: {converter.get('version')} at `{converter.get('commit')}`"  # type: ignore[union-attr]
        + (
            " (with uncommitted changes)"
            if converter.get("uncommitted_changes")
            else ""
        ),  # type: ignore[union-attr]
        f"- **Overrides**: `{' '.join(fingerprint['overrides']) or '(none)'}`",  # type: ignore[arg-type]
        f"- **Conversion**: {'completed' if conversion['completed'] else 'did not complete'}"  # type: ignore[index]
        f" in {conversion['seconds']} s; preprocessing "  # type: ignore[index]
        f"{'ran' if conversion['preprocessing_ran'] else 'did not run'}",  # type: ignore[index]
        f"- **Map**: `{json.dumps(fingerprint['map_census'], sort_keys=True)}`",
    ]
    notes = fingerprint.get("notes") or {}
    if "qc_checkers" in notes:  # type: ignore[operator]
        lines.append(
            f"- **qc checkers**: `{json.dumps(notes['qc_checkers'], sort_keys=True)}`"  # type: ignore[index]
            " -- an error count is a lower bound unless every checker completed"
        )
    for name, reason in (fingerprint.get("instruments_incomplete") or {}).items():  # type: ignore[union-attr]
        lines.append(f"- **Instrument `{name}` did not complete**: {reason}")
    lines += [
        "",
        "| finding | count | example (ids and numbers withheld) |",
        "|---|---:|---|",
    ]
    for row in fingerprint["findings"]:  # type: ignore[union-attr]
        example = (row["examples"] or [""])[0].replace("|", "\\|")
        lines.append(f"| `{row['finding']}` | {row['count']} | {example} |")
    if not fingerprint["findings"]:
        lines.append("| (none) | 0 | |")
    lines += ["", "## 🔁 Reproducer", ""]
    if repro:
        sanitization = repro["sanitization"]  # type: ignore[assignment]
        census_ = repro["map_census"]  # type: ignore[assignment]
        lines += [
            f"`repro.osm` (attached): {census_['lanelets']} lanelets, "  # type: ignore[index]
            f"converts in {repro['conversion_seconds']} s.",
            "",
            "```bash",
            str(repro["command"]),
            "```",
            "",
            f"It is a synthetic rebuild (level `{sanitization['level']}`): every id is "  # type: ignore[index]
            "renumbered, the excerpt is relocated and rotated, and coordinates are "
            f"perturbed by a {sanitization['warp_m']} m smooth warp and "  # type: ignore[index]
            f"{sanitization['jitter_sigma_m']} m jitter. The layout of these lanelets "  # type: ignore[index]
            "is real; no coordinate, id or elevation of the source map is.",
            "",
            "Findings on the reproducer:",
            "",
        ]
        for row in repro["findings"]:  # type: ignore[union-attr]
            lines.append(f"- `{row['finding']}` x{row['count']}")
    else:
        lines += [
            "None attached: this report is L0 only. The map cannot be shared.",
            "",
            "<!-- If a synthetic reproducer could not be produced, say why: the "
            "finding needs preprocessing, did not survive reduction, or did not "
            "survive the rebuild. Each of those is information. -->",
        ]
    lines += [
        "",
        "## ✅ Expected / ❌ Actual",
        "",
        "<!-- What the output should have been, and what it was. -->",
        "",
        "## ✅ Checklist",
        "",
        "- [ ] `leak-check` passed on everything attached or pasted here",
        "- [ ] No element id, coordinate, file path or screenshot of the source map "
        "appears in this issue",
        "- [ ] The owner of the source map agreed to any attached `repro.osm`",
        "",
    ]
    return "\n".join(lines)


def cmd_draft(args: argparse.Namespace, _overrides: List[str]) -> int:
    out = Path(getattr(args, "from"))
    public, private = out / "public", out / "private"
    fingerprint = _read_json(public / "fingerprint.json")
    repro_path = public / "repro.json"
    repro = _read_json(repro_path) if repro_path.is_file() else None
    (public / "issue.md").write_text(draft_issue(fingerprint, repro), encoding="utf-8")
    source = Path(_read_json(private / "locations.json")["source"])  # type: ignore[arg-type]
    reduced = private / "reduced.osm"
    checks = leak_check(public, source, reduced if reduced.is_file() else None)
    print(format_checks(checks))
    if not all(c.ok for c in checks):
        (public / "issue.md").unlink()
        return 1
    print(f"wrote {public / 'issue.md'}")
    return 0


def _hops(text: str) -> List[int]:
    return [int(part) for part in text.split(",") if part]


def main(argv: Optional[Sequence[str]] = None) -> int:
    # The qc bundle logs one INFO line per check to the root logger.
    logging.disable(logging.INFO)
    own, overrides = _split_overrides(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(
        description=(__doc__ or "").split("\n\n")[0],
        epilog="Hydra overrides for the converter go after a bare `--`.",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    def add(name: str, handler: object, summary: str) -> argparse.ArgumentParser:
        sub = commands.add_parser(name, help=summary, description=summary)
        sub.set_defaults(handler=handler)
        return sub

    sub = add("fingerprint", cmd_fingerprint, "convert MAP and write its L0 report")
    sub.add_argument("map")
    sub.add_argument("--out", required=True, help="directory for public/ and private/")
    sub.add_argument("--timeout", type=float, help="seconds for the conversion")

    sub = add("still-fails", cmd_still_fails, "exit 0 when MAP still shows the finding")
    sub.add_argument("map")
    sub.add_argument("--finding", required=True, help="instrument:code")
    sub.add_argument("--match", help="regex an example message must match")
    sub.add_argument("--timeout", type=float)

    sub = add("reduce", cmd_reduce, "shrink MAP to the lanelets that still show it")
    sub.add_argument("map")
    sub.add_argument("--from", required=True, help="the fingerprint's --out directory")
    sub.add_argument("--finding", required=True, help="instrument:code")
    sub.add_argument("--match", help="regex an example message must match")
    sub.add_argument(
        "--hops",
        type=_hops,
        default=[1, 2, 4],
        help="neighbour rounds to try, in order (default 1,2,4)",
    )
    sub.add_argument("--timeout", type=float, help="seconds per evaluation")

    sub = add("sanitize", cmd_sanitize, "rebuild the reduced map as a synthetic one")
    sub.add_argument("--from", required=True, help="the directory `reduce` wrote into")
    sub.add_argument("--finding", required=True, help="instrument:code")
    sub.add_argument("--match", help="regex an example message must match")
    sub.add_argument("--to-lat", type=float, default=DEFAULT_PLACEMENT[0])
    sub.add_argument("--to-lon", type=float, default=DEFAULT_PLACEMENT[1])
    sub.add_argument(
        "--rotate", type=float, help="degrees (default: drawn from the seed)"
    )
    sub.add_argument("--seed", type=int, help="default: random, recorded privately")
    sub.add_argument("--timeout", type=float)

    sub = add(
        "leak-check", cmd_leak_check, "verify public/ carries nothing of the source"
    )
    sub.add_argument("--from", required=True)

    sub = add("draft", cmd_draft, "write public/issue.md from the public documents")
    sub.add_argument("--from", required=True)

    args = parser.parse_args(own)
    return int(args.handler(args, overrides))


if __name__ == "__main__":
    sys.exit(main())
