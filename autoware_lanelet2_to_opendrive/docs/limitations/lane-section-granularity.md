# Lane Section Granularity and Cross-Slope

## Issue

Two structural features of OpenDRIVE are not produced by the converter:

1. A `<road>` never contains more than one `<laneSection>`.
2. No superelevation (cross-slope / banking) is emitted.

## One `<laneSection>` per road

Every `Lanes` element the converter builds is constructed with a
single-element list, `Lanes(lane_sections=[lane_section])`. This is true of
all four construction sites:

| Source | Role |
|--------|------|
| `opendrive/road.py` | Regular and junction roads |
| `divergence.py` | Divergence/merge roads |
| `opendrive/parking.py` | Synthetic parking-lot roads |
| `road_lanelet_geo_mapping.py` | Geometric cross-validation |

No code path appends, extends, or otherwise adds a second entry to
`lane_sections`.

### Consequence

OpenDRIVE allows a lane count to change *within* a road by opening a new
`<laneSection>` at the `s` where the change happens, typically with the
departing lane tapered to zero width. The converter cannot express this, so
a lane-count change appears as a boundary between two **`<road>`** elements,
connected through the road-level `<link><predecessor>` /
`<link><successor>` rather than through a tapered lane inside one road.

The mechanism is not that a group is split on lane count. Groups are built
from *lateral* adjacency only -- `util.py`'s grouping docstring says it
"does not consider longitudinal (following/previous) connections" -- so a
group never spans a lane-count change in the first place. (There is a
splitting pass, `split_groups_by_divergent_connections`, but it keys on
divergent downstream targets rather than on lane count.) The outcome is the
same; the cause is laterality, not a split.

Practical effects:

- More, shorter roads than a hand-authored OpenDRIVE map of the same network
  would contain.
- Lane drops and lane additions appear as road boundaries, not as width
  transitions.

## No superelevation

`<lateralProfile>` and its `<superelevation>` child are never written. The
converter contains no code that emits either element.

Lanelet2 carries no banking *attribute*, but that is not the same as there
being nothing to derive one from, and saying so on a limitations page would
frame a converter choice as a constraint of the input format. A lanelet's
left and right bounds are independent 3D linestrings and the converter
already reads their z values (`util.py` extracts `[p.x, p.y, p.z]` per
boundary point, and `opendrive/reference_line.py` builds the elevation
profile from them), so a cross-slope is computable as the height difference
across the carriageway over its width. It is not computed; nothing consumes
it today, and deriving it would need a decision about which of the two
bounds the reference line should bank around. The limit is that the
converter does not do this, not that it could not.

Vertical geometry *is* emitted: `<elevationProfile>` is generated from the
Lanelet2 z-coordinates (`opendrive/elevation.py`). Only the lateral
(cross-slope) component is absent, so every road surface is flat across its
width.

---

[← Back to Limitations Overview](index.md)
