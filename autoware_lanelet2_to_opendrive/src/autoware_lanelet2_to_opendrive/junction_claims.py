"""Merge junctions that claim the same road end.

A ``<connection>`` row of junction J with ``incomingRoad=R`` is a *claim* by J
on one end of R: ``contactPoint="start"`` means the connecting road starts at
the junction, so R ends there, and ``contactPoint="end"`` means R starts there.
A road end carries exactly one ``<link>`` slot, so when two junctions claim the
same end only one of them can be named there -- and every ``<connection>`` of
the other is then unreachable. A consumer arriving along R reads R's slot, gets
one junction id, and never looks at the second junction's table at all. The
connecting roads those rows point at are emitted, are geometrically fine, and
nothing ever resolves into them.

Three passes write that slot and none of them can see the conflict:

* :func:`~autoware_lanelet2_to_opendrive.divergence.apply_divergence_synthesis`
  writes ``<successor elementType="junction">`` for each synthetic
  divergence/merge junction *unconditionally*, before the real junction
  structure exists.
* :meth:`Road.set_incoming_road_junction_links` overwrites a *road*-typed slot
  with the junction link, but leaves a slot that already names another junction
  exactly as it found it -- so the real intersection silently loses to whichever
  synthetic junction got there first.
* Two synthetic junctions can claim one road end between themselves, in which
  case the later group overwrites the earlier one's slot.

The conflict is not a labelling accident: two junctions claiming one road end
are two descriptions of the same place, because they meet the same road at the
same point. This pass says so explicitly -- it merges each contested group into
a single junction and re-points everything that named a merged-away junction --
rather than picking a winner and discarding the loser's maneuvers, which would
delete real turns (on the reference map one of them is a 171.8 m connector).

Must run after every pass that writes junction links or appends
``<connection>`` rows, and before :meth:`Road.set_all_lane_links`, which reads
the final road links.
"""

from __future__ import annotations

import logging
from collections import defaultdict
from typing import Dict, List, Set, Tuple

from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint, ElementType
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    Connection,
    Junction,
    LaneLink,
)
from autoware_lanelet2_to_opendrive.opendrive.road import Road

log = logging.getLogger(__name__)

# A road end, identified by the road id and which of its two ends is meant.
RoadEnd = Tuple[int, ContactPoint]


def _claimed_end(connection: Connection) -> RoadEnd:
    """The end of ``connection.incoming_road`` this row claims.

    ``contactPoint="start"`` on the row means the connecting road *starts* at
    the junction, so the incoming road arrives there and it is its END that is
    claimed -- and the other way round for ``contactPoint="end"``.
    """
    end = (
        ContactPoint.END
        if connection.contact_point == ContactPoint.START
        else ContactPoint.START
    )
    return (connection.incoming_road, end)


def _contested_groups(junctions: List[Junction]) -> List[List[int]]:
    """Junction ids grouped by the road ends they share, lowest id first.

    Two junctions land in the same group when they claim the same road end,
    transitively: a chain of shared ends is one place, so merging it pairwise
    would leave a second conflict behind.

    Returns one sorted id list per group of size >= 2, ordered by the id that
    will win the merge, so the emitted XML does not depend on dict ordering.
    """
    claims: Dict[RoadEnd, Set[int]] = defaultdict(set)
    for junction in junctions:
        for connection in junction.connections:
            claims[_claimed_end(connection)].add(junction.id)

    parent: Dict[int, int] = {}

    def find(node: int) -> int:
        parent.setdefault(node, node)
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node

    def union(a: int, b: int) -> None:
        root_a, root_b = find(a), find(b)
        if root_a != root_b:
            # Lowest id always becomes the root, so the winner of a group is
            # its lowest id no matter what order the claims arrive in.
            if root_b < root_a:
                root_a, root_b = root_b, root_a
            parent[root_b] = root_a

    for claimants in claims.values():
        if len(claimants) < 2:
            continue
        members = sorted(claimants)
        for other in members[1:]:
            union(members[0], other)

    groups: Dict[int, Set[int]] = defaultdict(set)
    for jid in list(parent):
        groups[find(jid)].add(jid)
    return [sorted(members) for _, members in sorted(groups.items())]


def _absorb_connections(winner: Junction, losers: List[Junction]) -> None:
    """Move every loser's rows into ``winner``, de-duplicated and renumbered.

    Two junctions describing one place can hold the same ``(incoming road,
    connecting road, contact point)`` triple; keeping both would emit a
    duplicate ``<connection>``. Their ``<laneLink>`` sets are unioned instead,
    so no maneuver is lost to the de-duplication.
    """
    merged: Dict[Tuple[int, int, ContactPoint], Connection] = {}
    order: List[Tuple[int, int, ContactPoint]] = []
    for junction in [winner, *losers]:
        for connection in junction.connections:
            key = (
                connection.incoming_road,
                connection.connecting_road,
                connection.contact_point,
            )
            existing = merged.get(key)
            if existing is None:
                merged[key] = connection
                order.append(key)
                continue
            seen = {(link.from_lane, link.to_lane) for link in existing.lane_links}
            for link in connection.lane_links:
                if (link.from_lane, link.to_lane) not in seen:
                    existing.lane_links.append(
                        LaneLink(from_lane=link.from_lane, to_lane=link.to_lane)
                    )
                    seen.add((link.from_lane, link.to_lane))

    winner.connections = [merged[key] for key in order]
    for index, connection in enumerate(winner.connections):
        connection.id = index

    # ``Priority`` is frozen precisely so it can be de-duplicated; two
    # junctions describing one place can state the same precedence twice.
    seen_priorities = set(winner.priorities)
    for loser in losers:
        for controller_id in loser.controller_ids:
            if controller_id not in winner.controller_ids:
                winner.controller_ids.append(controller_id)
        for priority in loser.priorities:
            if priority not in seen_priorities:
                winner.priorities.append(priority)
                seen_priorities.add(priority)


def _repoint(roads: List[Road], remap: Dict[int, int]) -> Tuple[int, int]:
    """Re-point ``@junction`` attributes and junction-typed road links.

    Returns ``(roads re-homed, road links re-pointed)``.
    """
    rehomed = 0
    relinked = 0
    for road in roads:
        if road.junction is not None and road.junction in remap:
            road.junction = remap[road.junction]
            rehomed += 1
        if road.link is None:
            continue
        for link in (road.link.predecessor, road.link.successor):
            if (
                link is not None
                and link.element_type == ElementType.JUNCTION
                and link.element_id in remap
            ):
                link.element_id = remap[link.element_id]
                relinked += 1
    return rehomed, relinked


def merge_contested_junction_claims(
    *,
    all_roads: List[Road],
    junctions: List[Junction],
) -> int:
    """Merge every group of junctions that claim a road end between them.

    ``junctions`` is edited in place: the merged-away junctions are removed
    from the list, so the caller's list object stays the one the rest of the
    pipeline carries. ``all_roads`` is edited in place too -- a connecting road
    of a merged-away junction gets the surviving junction's id, and so does any
    road link that named one.

    The surviving junction of a group is its lowest id, which is deterministic
    and independent of the order the claims were written in.

    Args:
        all_roads: Regular + connecting roads; edited in place.
        junctions: All junctions; edited in place.

    Returns:
        The number of junctions merged away (0 when nothing was contested).
    """
    groups = _contested_groups(junctions)
    if not groups:
        return 0

    by_id: Dict[int, Junction] = {junction.id: junction for junction in junctions}
    remap: Dict[int, int] = {}
    for members in groups:
        winner_id, loser_ids = members[0], members[1:]
        winner = by_id[winner_id]
        losers = [by_id[loser_id] for loser_id in loser_ids]
        _absorb_connections(winner, losers)
        for loser_id in loser_ids:
            remap[loser_id] = winner_id
        log.info(
            "junction claim merge: junctions %s merged into %d "
            "(they claim the same road end)",
            loser_ids,
            winner_id,
        )

    rehomed, relinked = _repoint(all_roads, remap)
    junctions[:] = [j for j in junctions if j.id not in remap]

    log.info(
        "junction claim merge: %d junction(s) merged away across %d group(s); "
        "%d connecting road(s) re-homed, %d road link(s) re-pointed",
        len(remap),
        len(groups),
        rehomed,
        relinked,
    )
    return len(remap)
