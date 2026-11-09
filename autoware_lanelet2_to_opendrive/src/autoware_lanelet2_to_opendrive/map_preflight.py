"""Pre-flight structural inspection of a Lanelet2 OSM map.

``lanelet2.io.load`` refuses a structurally broken map with a single
``RuntimeError`` whose payload is an unordered wall of per-primitive lines::

    Errors ocurred while parsing Lanelet Map:
            - Error parsing primitive 2579841: Creating a regulatory element of type right_of_way failed: A maneuver must refer to at least one lanelet that has to yield!
            - Error parsing primitive 2579842: Creating a regulatory element of type right_of_way failed: ...
            ... 36 more lines ...
            - Error parsing primitive 2577778: Failed to get id 2579841 from map
            ...

Nothing in it says how many distinct problems there are, which lines are the
cause and which are the knock-on effect of it, or what a map author would have
to change.  On the map this module was written against, 38 lines described
*one* problem class with 19 instances plus its own cascade.

This module reads the OSM XML directly -- before, and independently of,
``lanelet2.io.load`` -- and reports the problems it recognises grouped by
class, with a count and a capped list of element ids per class.

It is deliberately a **diagnostic only**:

* it never edits the map and never hands the converter a patched copy;
* it never makes the converter tolerate input it would otherwise reject;
* a *blocking* finding corresponds to a constraint ``lanelet2.io.load``
  itself enforces, so it means the load *will* fail -- the report replaces
  the raw exception, it does not add a new class of rejection.  Each one is
  verified against a real ``lanelet2.io.load`` in ``test_map_preflight.py``;
* anything that is a smell but still loads is reported as *non-blocking* and
  conversion goes ahead, because rejecting a map that converts today would
  be a worse failure than leaving a smell unenforced.

Usage::

    report = inspect_lanelet2_osm(Path("map.osm"))
    if not report.ok:
        print(report.format())
        sys.exit(1)
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Sequence, Set, Tuple

import lxml.etree as ET

from .config import DEFAULT_CONFIG

#: Roles a ``right_of_way`` regulatory element may carry, and the one that
#: lanelet2 requires at least one of.  See
#: ``lanelet2_core/src/RightOfWay.cpp``: constructing the element throws
#: "A maneuver must refer to at least one lanelet that has to yield!" when no
#: member carries the ``yield`` role.
_YIELD_ROLE = "yield"

#: Member roles that carry a lanelet's two boundaries.  ``lanelet2`` rejects a
#: ``type=lanelet`` relation that is missing either of them.
_LANELET_BOUND_ROLES = ("left", "right")


@dataclass(frozen=True)
class Finding:
    """One class of problem, with every element that exhibits it.

    Attributes:
        code: Stable machine-readable identifier for the problem class.
        summary: One line saying what is wrong, in the map author's terms.
        consequence: What ``lanelet2.io.load`` does about it.
        remedy: What has to change in the map for the load to succeed.
        element_ids: OSM ids of every element in this class, in file order.
        blocking: True when the map provably does not load because of this,
            verified against ``lanelet2.io.load`` in
            ``test_map_preflight.py``.  A non-blocking class is reported and
            conversion continues: it would be worse to reject a map that
            converts today than to leave a smell unenforced.
    """

    code: str
    summary: str
    consequence: str
    remedy: str
    element_ids: Tuple[str, ...]
    blocking: bool = True

    @property
    def count(self) -> int:
        """Number of elements exhibiting this problem."""
        return len(self.element_ids)


@dataclass(frozen=True)
class PreflightReport:
    """Result of inspecting one map.

    Attributes:
        path: The inspected file.
        findings: One entry per problem class that has at least one element,
            blocking and non-blocking alike.
        counts: Primitive census of the file, for context in the report.
        xml_error: The parser message when the file is not well-formed XML,
            in which case no check ran; ``None`` otherwise.
    """

    path: Path
    findings: Tuple[Finding, ...] = ()
    counts: Dict[str, int] = field(default_factory=dict)
    xml_error: str | None = None

    @property
    def ok(self) -> bool:
        """True when nothing was found that stops the map loading.

        Only *blocking* findings count.  A non-blocking one is reported and
        conversion goes ahead, and a file this module could not parse is
        ``ok`` too: Lanelet2's own reader is laxer than strict XML
        (``walkway_mini.osm`` in this repository carries a ``--`` inside a
        comment, which lxml rejects and lanelet2 accepts), so refusing to
        convert on that basis would reject maps that work today.
        :attr:`xml_error` carries the parser message for the log instead.
        """
        return not any(finding.blocking for finding in self.findings)

    @property
    def inspected(self) -> bool:
        """True when the file was parsed far enough for the checks to mean anything."""
        return self.xml_error is None

    @property
    def total_elements(self) -> int:
        """Total number of problem elements across all classes."""
        return sum(f.count for f in self.findings)

    def format(self, max_ids: int | None = None) -> str:
        """Render the report as operator-facing text.

        Args:
            max_ids: Maximum number of element ids printed per problem class.
                The remainder is summarised as a count.  Defaults to
                ``DEFAULT_CONFIG.preflight.max_reported_ids``.

        Returns:
            A multi-line string.  Empty findings render as a single OK line.
        """
        cap = DEFAULT_CONFIG.preflight.max_reported_ids if max_ids is None else max_ids
        census = "  ".join(f"{k}={v}" for k, v in sorted(self.counts.items()))
        head = [f"Lanelet2 pre-flight: {self.path}", f"  {census}"]
        if not self.inspected:
            return "\n".join(
                head
                + [
                    f"  not inspected: {self.xml_error}",
                    "  (lanelet2's reader may still accept this file; "
                    "conversion continues)",
                ]
            )
        if not self.findings:
            return "\n".join(head + ["  no structural problems found"])

        blocking = sum(1 for f in self.findings if f.blocking)
        lines = head + [
            f"  {len(self.findings)} problem class(es) "
            f"({blocking} that stop the map loading), "
            f"{self.total_elements} element(s) total",
            "",
        ]
        for f in self.findings:
            severity = "stops the load" if f.blocking else "does not stop the load"
            lines.append(f"[{f.code}] {f.count} element(s) -- {severity}")
            lines.append(f"  what : {f.summary}")
            lines.append(f"  load : {f.consequence}")
            lines.append(f"  fix  : {f.remedy}")
            shown = f.element_ids[:cap]
            suffix = (
                f" ... and {f.count - len(shown)} more" if f.count > len(shown) else ""
            )
            lines.append(f"  ids  : {', '.join(shown)}{suffix}")
            lines.append("")
        return "\n".join(lines).rstrip()


class MapPreflightError(Exception):
    """Raised when the pre-flight found problems that stop the map loading.

    The message is the formatted report, so a caller that only prints the
    exception still shows the named problem classes and their counts.

    Attributes:
        report: The :class:`PreflightReport` that produced the message.
    """

    def __init__(self, report: "PreflightReport") -> None:
        super().__init__(report.format())
        self.report = report


#: Stands in for a primitive with no ``id`` attribute, so a report naming it
#: is still printable.  Lanelet2 cannot read such an element either, but
#: refusing the map over it would be a new rejection rather than a name for
#: one the parser already makes, which is not what this module is for.
_NO_ID = "<no id>"


def _tags(element: ET._Element) -> Dict[str, str]:
    """Return an OSM element's ``<tag k= v=>`` children as a dict."""
    return {t.get("k"): t.get("v") for t in element.findall("tag")}


def _iter_primitives(path: Path) -> Iterator[ET._Element]:
    """Yield every ``<node>``/``<way>``/``<relation>``, then free it.

    Streaming keeps peak memory independent of map size; the maps this runs
    on reach 100 MB, where holding the whole DOM costs well over a gigabyte.
    Each element is cleared once yielded, so callers must extract everything
    they need before asking for the next one.

    Entity substitution is switched off because the input is third-party and
    may be malformed by definition; lxml already refuses network access and
    oversized trees by default.
    """
    context = ET.iterparse(
        str(path),
        events=("end",),
        tag=("node", "way", "relation"),
        resolve_entities=False,
        no_network=True,
    )
    for _, element in context:
        yield element
        element.clear()
        while element.getprevious() is not None:
            del element.getparent()[0]


def _finding(
    code: str,
    summary: str,
    consequence: str,
    remedy: str,
    ids: Iterable[str],
    blocking: bool = True,
) -> Finding | None:
    """Build a :class:`Finding`, or ``None`` when no element matched."""
    ordered = tuple(ids)
    if not ordered:
        return None
    return Finding(
        code=code,
        summary=summary,
        consequence=consequence,
        remedy=remedy,
        element_ids=ordered,
        blocking=blocking,
    )


def inspect_lanelet2_osm(path: Path) -> PreflightReport:
    """Inspect a Lanelet2 OSM file for problems that stop it loading.

    The file is parsed as plain XML, so this works on maps that
    ``lanelet2.io.load`` refuses outright.

    Args:
        path: Path to the ``.osm`` file.

    Returns:
        A :class:`PreflightReport`.  ``report.ok`` is True when none of the
        implemented checks matched; that is *not* a guarantee that the map
        loads, only that it does not fail for any reason this module knows
        how to name.  When the file is not well-formed XML, the report
        carries :attr:`PreflightReport.xml_error` and no findings, because
        lanelet2's reader accepts constructs that lxml rejects.

    Raises:
        FileNotFoundError: If *path* does not exist.
    """
    if not path.exists():
        raise FileNotFoundError(f"Lanelet2 file not found: {path}")

    node_ids: Set[str] = set()
    way_ids: Set[str] = set()
    relation_ids: Set[str] = set()
    duplicate_ids: List[str] = []

    row_without_yield: List[str] = []
    lanelet_missing_bound: List[str] = []
    degenerate_ways: List[str] = []
    way_node_refs: List[Tuple[str, List[str]]] = []
    # Relation members are resolved in a second pass: a relation may refer
    # forwards to an id that appears later in the file, so nothing can be
    # called dangling until every id has been seen.
    relation_member_refs: List[Tuple[str, List[Tuple[str, str]]]] = []

    try:
        for element in _iter_primitives(path):
            # An element with no `id` cannot be reported by id, and `None` in
            # the id lists makes `format`'s `", ".join(...)` raise TypeError
            # -- which on the *non-blocking* path kills a map that converts,
            # inside the branch written to avoid refusing one.  A placeholder
            # keeps the report printable and says what is wrong.
            oid = element.get("id") or _NO_ID
            if element.tag == "node":
                if oid in node_ids:
                    duplicate_ids.append(oid)
                node_ids.add(oid)
                continue

            if element.tag == "way":
                if oid in way_ids:
                    duplicate_ids.append(oid)
                way_ids.add(oid)
                refs = [nd.get("ref") for nd in element.findall("nd")]
                if len(refs) < 2:
                    degenerate_ways.append(oid)
                way_node_refs.append((oid, refs))
                continue

            if oid in relation_ids:
                duplicate_ids.append(oid)
            relation_ids.add(oid)
            tags = _tags(element)
            members = element.findall("member")
            roles = [m.get("role") for m in members]

            if (
                tags.get("type") == "regulatory_element"
                and tags.get("subtype") == "right_of_way"
                and _YIELD_ROLE not in roles
            ):
                row_without_yield.append(oid)

            if tags.get("type") == "lanelet" and not all(
                role in roles for role in _LANELET_BOUND_ROLES
            ):
                lanelet_missing_bound.append(oid)

            relation_member_refs.append(
                (oid, [(m.get("type"), m.get("ref")) for m in members])
            )
    except ET.XMLSyntaxError as exc:
        # Not well-formed XML. Lanelet2's own reader may still accept it, so
        # this is reported, not treated as a finding: see PreflightReport.ok.
        return PreflightReport(path=path, xml_error=str(exc))

    known: Dict[str, Set[str]] = {
        "node": node_ids,
        "way": way_ids,
        "relation": relation_ids,
    }
    dangling_relation_members = [
        rid
        for rid, members in relation_member_refs
        if any(
            known.get(mtype) is not None and ref not in known[mtype]
            for mtype, ref in members
        )
    ]
    dangling_way_nodes = [
        wid for wid, refs in way_node_refs if any(r not in node_ids for r in refs)
    ]

    candidates = [
        _finding(
            "right_of_way_without_yield",
            "a right_of_way regulatory element has no member with role "
            "'yield', so the element never says who gives way (on the map "
            "this was first found on every member was tagged 'right_of_way', "
            "but the check is only the absence of a 'yield' role)",
            "rejected with 'A maneuver must refer to at least one lanelet that "
            "has to yield!'; every lanelet referring to the element then fails "
            "too with 'Failed to get id ... from map'",
            "give the yielding lanelet the role 'yield' -- normally the "
            "element's first member, and NOT the lanelet that owns the "
            "element, which is the one with priority (measured on this "
            "repository's reference map: 85 of 85 right_of_way elements are "
            "owned by a lanelet holding the 'right_of_way' role) -- or delete "
            "the element if nothing yields there",
            row_without_yield,
        ),
        _finding(
            "lanelet_missing_bound",
            "a type=lanelet relation is missing its 'left' or 'right' member",
            "rejected: a lanelet cannot be constructed without both boundaries",
            "add the missing boundary way as a member with that role",
            lanelet_missing_bound,
        ),
        _finding(
            "dangling_member_ref",
            "a relation refers to an id that is not in this file",
            "rejected with 'Failed to get id ... from map'",
            "add the missing primitive, or drop the member that refers to it",
            dangling_relation_members,
        ),
        _finding(
            "way_references_missing_node",
            "a way lists an <nd ref> that is not in this file",
            "rejected: the linestring cannot be built",
            "add the missing node, or drop the <nd> that refers to it",
            dangling_way_nodes,
        ),
        _finding(
            "degenerate_way",
            "a way has fewer than two nodes, so it has no extent",
            "accepted by the parser but unusable as a lanelet boundary",
            "give the way at least two distinct nodes, or remove it",
            degenerate_ways,
            blocking=False,
        ),
        _finding(
            "duplicate_primitive_id",
            "the same id appears more than once in one primitive layer",
            "the later definition silently replaces the earlier one",
            "renumber the duplicate",
            duplicate_ids,
            blocking=False,
        ),
    ]

    return PreflightReport(
        path=path,
        findings=tuple(f for f in candidates if f is not None),
        counts={
            "nodes": len(node_ids),
            "ways": len(way_ids),
            "relations": len(relation_ids),
        },
    )


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point: inspect a map and exit non-zero when it is broken.

    Args:
        argv: Argument vector, defaulting to ``sys.argv[1:]``.

    Returns:
        ``1`` when the map has a finding that stops it loading, ``0``
        otherwise.  A non-blocking finding is printed and still exits ``0``,
        so this can gate a pipeline without rejecting maps that convert.
    """
    parser = argparse.ArgumentParser(
        prog="map-preflight",
        description="Report structural problems in a Lanelet2 OSM map.",
    )
    parser.add_argument("map_path", type=Path, help="Lanelet2 .osm file")
    parser.add_argument(
        "--max-ids",
        type=int,
        default=DEFAULT_CONFIG.preflight.max_reported_ids,
        help="maximum element ids printed per problem class",
    )
    args = parser.parse_args(argv)

    report = inspect_lanelet2_osm(args.map_path)
    print(report.format(max_ids=args.max_ids))
    return 0 if report.ok else 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
