# Lane Section Granularity and Cross-Slope

## Issue

Two structural features of OpenDRIVE are not produced by the converter:

1. A `<road>` never contains more than one `<laneSection>`.
2. No superelevation (cross-slope / banking) is emitted.

## One `<laneSection>` per road

Every `Lanes` element the converter builds is constructed with a
single-element list, `Lanes(lane_sections=[lane_section])`. This is true of
all four construction sites:

| Source | Path |
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
departing lane tapered to zero width. The converter cannot express this.
Where the number of lanes changes, the lanelet group is split and the change
becomes a **new `<road>`**, connected to its neighbour through the road-level
`<link><predecessor>` / `<link><successor>` elements rather than through a
tapered lane inside one road.

Practical effects:

- More, shorter roads than a hand-authored OpenDRIVE map of the same network
  would contain.
- Lane drops and lane additions appear as road boundaries, not as width
  transitions.

## No superelevation

`<lateralProfile>` and its `<superelevation>` child are never written. The
converter contains no code that emits either element, and Lanelet2 carries no
banking attribute to derive one from.

Vertical geometry *is* emitted: `<elevationProfile>` is generated from the
Lanelet2 z-coordinates (`opendrive/elevation.py`). Only the lateral
(cross-slope) component is absent, so every road surface is flat across its
width.

---

[← Back to Limitations Overview](index.md)
