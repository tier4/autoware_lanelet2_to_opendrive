# Pedestrian Network Connectivity

## Issue

`walkway` lanelets are emitted as **isolated one-lane roads**: no road-level
`<link>`, no lane-level `<link>`, and no shared boundary with any neighbouring
road. A pedestrian cannot be routed from a walkway to anything else through
the OpenDRIVE topology.

This is a **limitation of the input map data**, not of the conversion. It
cannot be fixed in the converter without guessing.

## Cause

Lateral grouping and lane-link resolution both derive adjacency from
*shared boundary geometry* — a lanelet's boundary linestring **being** its
neighbour's (see `build_boundary_lateral_adjacency` and
`build_boundary_longitudinal_adjacency`). That is the relationship the
Lanelet2 map specification guarantees for lanelets that belong together.

`walkway` lanelets in the reference maps share no such geometry with
anything at all.

## Measurements

Taken directly from the two reference maps.

### What each subtype shares

| Subtype | Count (nishishinjuku / odaiba) | Shares a boundary **linestring** with | Shares a boundary **endpoint node** with | Fully detached |
|---|---|---|---|---|
| `walkway` | 8 / 180 | nothing | nothing | **8 / 8 and 180 / 180** |
| `crosswalk` | 84 / 793 | nothing | 1 `road` / 4 `crosswalk` | 83 / 84 and 789 / 793 |
| `pedestrian_lane` | 0 / 2,192 | `road`: 2,188 | `road` 2,192, `pedestrian_lane` 2,101 | 0 |
| `road_shoulder` | 3 / 1,984 | `road`: 3 / 1,984 | `road_shoulder` 3 / 1,731 | 0 |
| `bicycle_lane` | 0 / 49 | `road`: 49 | `bicycle_lane` 45 | 0 |

`pedestrian_lane`, `road_shoulder` and `bicycle_lane` are the opposite case:
100% of them share a boundary linestring with a road, so they are folded into
that road as extra lanes, and most share endpoint nodes with a same-class
peer, so their longitudinal succession is derivable. Only `walkway` is
stranded.

### How far apart the pedestrian surfaces are

Nearest end-to-end distance between two *distinct* pedestrian lanelets
(nishishinjuku, cross-section midpoints):

| percentile | distance |
|---|---|
| min | 1.54 m |
| p25 | 8.24 m |
| median | 12.41 m |
| p90 | 52.29 m |

**No pair is closer than 0.5 m.** There is no near-coincidence to snap to.

### Why proximity to roads is not a usable signal either

Walkways do lie close to roads — 0.00 m to 0.84 m from the nearest road
lanelet on nishishinjuku. But measuring each walkway boundary polyline
against the nearest road boundary **along its whole length**, only **2 of 8**
stay within 0.5 m; the rest reach 1.2 m to 3.7 m. They are separate footpath
geometry running near roads, not duplicated road boundaries.

Attaching them by proximity would need a tolerance greater than **3.7 m** —
wider than a lane. At that tolerance the nearest surface is frequently the
wrong one, which would produce silent wrong-neighbour links.

### A pedestrian routing graph is not an alternative

A `Participants.Pedestrian` `RoutingGraph` built on nishishinjuku yields
**zero** predecessors, successors and lateral neighbours for *every* subtype,
`walkway` and `crosswalk` included.

## Impact

- 8 walkway roads on nishishinjuku and 180 walkway lanes on odaiba carry no
  links. On odaiba this is part of 241 pedestrian-only roads that are
  road-level isolated.
- The walkways are still **emitted correctly**: `<lane type="sidewalk">` with
  the right geometry, width, curb `<height>` and `<access>`. They are
  walkable islands, not missing surfaces.
- For CARLA specifically this matters less than it appears. CARLA builds its
  pedestrian navmesh from the **generated mesh**, keyed on lane type, not from
  OpenDRIVE `<link>`; a `sidewalk` lane of non-zero width produces walkable
  surface regardless of its links. What the missing links cost is the
  `carla.Waypoint` / `get_topology` view — scenario placement and `wp.next()`
  traversal. See [CARLA ↔ OpenDRIVE ↔ Lanelet2 mapping](../carla_opendrive_lanelet2_mapping.md).

!!! warning "Source-derived, not run"
    The CARLA behaviour described above is derived from CARLA 0.10.0 source
    and from the pinned client wheel in `carla_wheels/`. No CARLA server was
    run to confirm it, and no walker was observed to spawn.

## On the `walkway` subtype definition

The pilot.auto vector-map requirements describe `walkway` as the ±3 m region
where a road intersects a crosswalk. **That is not what these maps contain.**
On nishishinjuku the 8 walkways are 8.5 m to 59.7 m from the nearest
crosswalk, and 0.00 m to 0.84 m from the nearest road. They read as ordinary
roadside footpaths, which is why they are treated as standalone sidewalks
rather than as crosswalk approaches.

## Workaround

None on the converter side, by design. The fix belongs in the map:

- Have walkway lanelets **share the boundary linestring** of the lanelet they
  adjoin, as the Lanelet2 specification expects for lanelets that belong
  together. The existing attachment pass then folds them into that road with
  no converter change at all.
- Where walkways should form a continuous footpath, have consecutive
  walkways **share their cross-section points**, which
  `build_boundary_longitudinal_adjacency` already reads.

Both are data changes that make the relationship explicit rather than
inferred. The converter deliberately does not guess it: a proximity
threshold wide enough to catch these walkways is wide enough to catch the
wrong neighbour.
