"""Match the roads of two conversions of the same map by their source lanelets.

Road ids are not a key that survives a change of road grouping. They are
assigned in emission order, so a change that adds, removes or regroups a road
renumbers every road after it, and the same id then names two unrelated roads
in the two outputs (see the module docstring of ``xodr_digest``). Lining two
outputs up by road id across such a change compares roads that have nothing to
do with each other, in both directions: a changed field is not evidence the
road changed, and an unchanged one is not evidence it did not.

What does survive is what a road was *built from*. The companion
``.mapping.json`` records, for every lanelet, the road it was emitted on. Two
roads are the same road to the degree that they are built from the same
lanelets, measured here by the Jaccard index of their lanelet-id sets. There
is no geometric threshold and no free parameter.

Two things have to be read from the mapping for that key to hold:

``preprocessing_log``
    A ``merge_operations`` entry replaces the listed lanelets by one lanelet
    under a new synthetic id, so one side names an id the other has never
    heard of. Every synthetic id is expanded back to the lanelets it was merged
    from before the sets are compared. Without this the merged roads are
    exactly the ones that go unmatched: on Nishi-Shinjuku, 6 of 640 roads, one
    per ``merge_operations`` entry.

Additional road assignments
    ``lanelet_to_road_and_lane`` names one road per lanelet. Where a mapping
    also records that a lanelet belongs to further roads
    (``lanelet_additional_road_lanes``, ``chain_merged_roads``,
    ``exit_fork_duplicate_roads``), those are part of a road's composition and
    are folded in. A mapping without these fields is read as it stands.

What it cannot do: a road with no source lanelet (a synthetic connector,
listed in ``skipped_synthetic_roads``) has nothing to match on. Such roads are
returned under ``not_matchable`` and counted in the summary, never dropped: a
correspondence that left them out would read as "every road was compared"
when a class of roads was never looked at.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Any, Iterable, Mapping

_MERGE_OPERATION = "merge"


def _numeric(road_ids: Iterable[str]) -> list[str]:
    """Sort road ids, given as strings, in numeric order."""
    return sorted(road_ids, key=int)


def _merge_sources(mapping: Mapping[str, Any]) -> dict[str, list[str]]:
    """Return ``synthetic lanelet id -> the lanelet ids it was merged from``."""
    log = mapping.get("preprocessing_log") or {}
    sources: dict[str, list[str]] = {}
    for entry in log.get("entries") or []:
        if entry.get("operation") != _MERGE_OPERATION:
            continue
        inputs = [str(lanelet) for lanelet in entry.get("input_ids") or []]
        for output in entry.get("output_ids") or []:
            sources[str(output)] = inputs
    return sources


def _original_lanelets(lanelet: str, sources: Mapping[str, list[str]]) -> set[str]:
    """Expand a lanelet id to the ids of the input map it stands for.

    A merge output can itself be merged again, so the expansion is repeated
    until only ids that are not a merge output remain. ``seen`` bounds the walk
    on a log that names an id as its own source.
    """
    originals: set[str] = set()
    pending = [lanelet]
    seen: set[str] = set()
    while pending:
        current = pending.pop()
        if current in seen:
            continue
        seen.add(current)
        if current in sources:
            pending.extend(sources[current])
        else:
            originals.add(current)
    return originals


def road_compositions(mapping: Mapping[str, Any]) -> dict[str, frozenset[str]]:
    """Return ``road id -> the input-map lanelets that road was built from``.

    Road ids and lanelet ids are returned as strings, the way a mapping
    document spells its keys.
    """
    sources = _merge_sources(mapping)
    roads: dict[str, set[str]] = defaultdict(set)

    def add(road: Any, lanelet: Any) -> None:
        roads[str(road)].update(_original_lanelets(str(lanelet), sources))

    for lanelet, (road, _lane) in mapping["lanelet_to_road_and_lane"].items():
        add(road, lanelet)
    for lanelet, pairs in (mapping.get("lanelet_additional_road_lanes") or {}).items():
        for road, _lane in pairs:
            add(road, lanelet)
    for key in ("chain_merged_roads", "exit_fork_duplicate_roads"):
        for road, lanelets in (mapping.get(key) or {}).items():
            for lanelet in lanelets:
                add(road, lanelet)
    return {road: frozenset(lanelets) for road, lanelets in roads.items()}


def _jaccard(a: frozenset[str], b: frozenset[str]) -> float:
    union = len(a | b)
    return len(a & b) / union if union else 0.0


def _candidates(
    lanelets: frozenset[str],
    other: Mapping[str, frozenset[str]],
    lanelet_to_other_roads: Mapping[str, set[str]],
) -> list[tuple[str, float]]:
    """Roads on the other side sharing a lanelet, best match first.

    Ties are broken by numeric road id so the result does not depend on
    dictionary order.
    """
    shared: set[str] = set()
    for lanelet in lanelets:
        shared.update(lanelet_to_other_roads.get(lanelet, ()))
    scored = [(road, _jaccard(lanelets, other[road])) for road in shared]
    scored.sort(key=lambda item: (-item[1], int(item[0])))
    return scored


def _lanelet_index(roads: Mapping[str, frozenset[str]]) -> dict[str, set[str]]:
    index: dict[str, set[str]] = defaultdict(set)
    for road, lanelets in roads.items():
        for lanelet in lanelets:
            index[lanelet].add(road)
    return index


def _not_matchable(
    mapping: Mapping[str, Any], roads: Mapping[str, frozenset[str]]
) -> list[str]:
    """Roads the mapping lists as having no source lanelet."""
    listed = {str(road) for road in mapping.get("skipped_synthetic_roads") or []}
    return _numeric(listed - set(roads))


def match_roads(
    baseline_mapping: Mapping[str, Any], comparison_mapping: Mapping[str, Any]
) -> dict[str, Any]:
    """Return the road correspondence between two mapping documents.

    Both arguments are parsed ``.mapping.json`` documents of the same input
    map. The result has:

    ``baseline_to_comparison``
        For every baseline road with a source lanelet: ``best_match`` (the
        comparison road with the highest Jaccard index, ``None`` if no road
        shares a lanelet), ``jaccard``, ``shared_lanelets``,
        ``baseline_lanelets``, ``comparison_lanelets``, ``candidates`` (every
        comparison road sharing a lanelet, best first) and ``symmetric``
        (whether the comparison road's own best match is this road).
    ``unmatched_baseline`` / ``new_in_comparison``
        Roads sharing no lanelet with any road on the other side.
    ``many_to_one``
        Groups of baseline roads whose best match is the same comparison road:
        roads that were merged.
    ``one_to_many``
        ``[baseline road, [comparison roads]]`` for a baseline road whose
        lanelets appear on several comparison roads: a road that was split, or
        a stage that was duplicated.
    ``not_matchable``
        Per side, the roads listed in ``skipped_synthetic_roads``. They have
        no source lanelet and are in none of the lists above.
    ``summary``
        Counts of the above, and ``is_identity_map``: every baseline road
        matches the comparison road of the same id with a Jaccard index of 1.
    """
    baseline = road_compositions(baseline_mapping)
    comparison = road_compositions(comparison_mapping)
    lanelet_to_comparison = _lanelet_index(comparison)
    lanelet_to_baseline = _lanelet_index(baseline)

    comparison_best = {}
    for road, lanelets in comparison.items():
        ranked = _candidates(lanelets, baseline, lanelet_to_baseline)
        comparison_best[road] = ranked[0][0] if ranked else None

    matches: dict[str, dict[str, Any]] = {}
    for road in _numeric(baseline):
        lanelets = baseline[road]
        ranked = _candidates(lanelets, comparison, lanelet_to_comparison)
        best = ranked[0][0] if ranked else None
        matches[road] = {
            "best_match": best,
            "jaccard": ranked[0][1] if ranked else 0.0,
            "shared_lanelets": len(lanelets & comparison[best]) if best else 0,
            "baseline_lanelets": len(lanelets),
            "comparison_lanelets": len(comparison[best]) if best else 0,
            "candidates": [[candidate, score] for candidate, score in ranked],
            "symmetric": best is not None and comparison_best[best] == road,
        }

    unmatched = [road for road, entry in matches.items() if entry["best_match"] is None]
    new = _numeric(road for road, best in comparison_best.items() if best is None)

    by_best: dict[str, list[str]] = defaultdict(list)
    for road, entry in matches.items():
        if entry["best_match"] is not None:
            by_best[entry["best_match"]].append(road)
    many_to_one = sorted(
        (group for group in by_best.values() if len(group) > 1),
        key=lambda group: int(group[0]),
    )
    one_to_many = [
        [road, _numeric(candidate for candidate, _ in entry["candidates"])]
        for road, entry in matches.items()
        if len(entry["candidates"]) > 1
    ]

    not_matchable = {
        "baseline": _not_matchable(baseline_mapping, baseline),
        "comparison": _not_matchable(comparison_mapping, comparison),
    }
    perfect = sum(1 for entry in matches.values() if entry["jaccard"] == 1.0)
    return {
        "baseline_to_comparison": matches,
        "unmatched_baseline": unmatched,
        "new_in_comparison": new,
        "many_to_one": many_to_one,
        "one_to_many": one_to_many,
        "not_matchable": not_matchable,
        "summary": {
            "baseline_roads": len(baseline),
            "comparison_roads": len(comparison),
            "matched": len(matches) - len(unmatched),
            "perfect_matches": perfect,
            "symmetric_matches": sum(
                1 for entry in matches.values() if entry["symmetric"]
            ),
            "unmatched_baseline": len(unmatched),
            "new_in_comparison": len(new),
            "many_to_one_groups": len(many_to_one),
            "one_to_many_roads": len(one_to_many),
            "not_matchable_baseline": len(not_matchable["baseline"]),
            "not_matchable_comparison": len(not_matchable["comparison"]),
            "is_identity_map": bool(matches)
            and all(
                entry["best_match"] == road and entry["jaccard"] == 1.0
                for road, entry in matches.items()
            ),
        },
    }
