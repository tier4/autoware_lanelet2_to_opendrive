# Test maps

| file | lanelets | what it is for |
|---|---|---|
| `nishishinjuku.osm` | 979 | the project's reference map; most integration tests convert it |
| `junction_cluster.osm` | 190 | **not committed.** A junction cluster with geometry from a real urban map — see below |
| `junction_fanin_mini.osm` | 9 | a one-lane and a two-lane road merging through three adjacent connectors |
| `junction_fanout_mini.osm` | 9 | three adjacent connectors leading to a one-lane and a two-lane road |
| `parking_lot_mini.osm` | — | parking-lot areas and their stalls |
| `short_lanelet_mini.osm` | — | lanelets shorter than one paramPoly3 segment |
| `speed_change_mini.osm` | — | a speed limit changing mid-road |
| `walkway_mini.osm` | — | walkway and sidewalk lane typing |
| `tmerc_mini/`, `tmerc_mini_k9999/` | — | TransverseMercator projection round-trip |

`nishishinjuku_carla.{xodr,mapping.json,preprocessed.osm}` are generated, not
committed, and are bound together by a SHA-256 in the sidecar: mixing files
from different conversion runs makes the loader return `None` and surfaces
later as an unrelated assertion failure. Run `git clean -fdX` in this
directory before measuring anything against them.

## `junction_cluster.osm`

Converted with `map=junction_cluster`. It exists because the `*_mini`
fixtures are drawn by hand on a 20 m grid, and a grid cannot produce what a
surveyed network has: lane boundaries digitised at irregular intervals, two
branches of a fork whose start points differ by centimetres rather than
coinciding exactly, and junctions with more than a handful of connectors.
A defect in how a fork's start points are reconciled is a provable no-op on a
grid-drawn fixture, because there the two branches share the same boundary
vertex.

### It is not committed

The geometry comes from a real urban map that cannot be redistributed here, so
the file is absent from this repository and
`test_real_derived_cluster_satisfies_invariants` skips. The two tools that
produce it are committed, and the invocation below is exact; run it against a
source map you hold to enable that check locally.

Whether a rebuilt excerpt may be committed is a decision for the owner of the
source map, not a property the rebuild can establish on its own.

### Where it comes from, and what was changed

It is a 400 m junction cluster cut out of a real urban Lanelet2 map of about
19 000 lanelets, then rebuilt. Two committed tools do it, in order:

```bash
python tools/crop_lanelet2_map.py <source>.osm /tmp/excerpt.osm \
    --x <easting> --y <northing> --w 400 --hops 1

python tools/rebuild_as_synthetic.py /tmp/excerpt.osm junction_cluster.osm \
    --rotate 37 --to-lat 35.688045 --to-lon 139.692086
```

`crop_lanelet2_map.py` takes every lanelet whose centroid is in the box, then
one round of routing and adjacency neighbours so a junction keeps the approach
and exit roads that give it its shape, then the regulatory-element and
way/node closure that makes the result loadable on its own. Of 119 regulatory
elements reachable from the kept lanelets, 95 survive; 24 refer to a lanelet
outside the box and are dropped, along with the reference to them.

`rebuild_as_synthetic.py` then renumbers every node, way and relation from a
fresh counter, applies a rigid transform (rotation about the excerpt's
centroid, then placement at a new centre), **perturbs every coordinate**, and
drops every tag whose value is an id in the source — reporting each such tag
by name and count rather than silently.

### The perturbation, and why these amplitudes

Relocating and renumbering leaves the geometry itself untouched: locally it
would still be the surveyed shape. So the coordinates are perturbed too, by
two components sized for different jobs.

| | value | why |
|---|---|---|
| smooth warp | 0.25 m peak per axis, 150 m wavelength | Displaces the network by up to 0.31 m (mean 0.14 m) while moving each neighbourhood almost as a unit. Its gradient is bounded by `warp·π/wavelength` = **0.52%**, so a segment's length cannot change by more than that from the warp — check it against the vertex-spacing table below. Distances between features hundreds of metres apart stop matching the source. |
| per-position jitter | σ = 0.02 m | Breaks the correspondence at the individual vertex, which the warp alone leaves locally intact. Two centimetres is the order of an HD survey's own precision, so a perturbed vertex is where a second survey of the same road might plausibly have put it — enough that the value is not the recorded one, small enough not to invent geometry. |

Both are a function of the vertex's **position**, not its identity. That is
load-bearing: two vertices at the same point get the same displacement, so a
vertex shared between two ways stays one point and the 33 pairs of
distinct-but-exactly-coincident vertices in the source stay exactly
coincident. Keying on identity instead destroyed all 33 — a real-map
degeneracy the converter has to cope with — which is why it does not.

The whole thing is deterministic from `--seed`, so the fixture is
reproducible.

### What the perturbation preserved, measured

Excerpt (before) against the committed fixture (after), over the same 3906
boundary segments:

| | before | after |
|---|---|---|
| boundary segments | 3906 | 3906 |
| vertex gap: median / p95 / max (m) | 1.195 / 8.305 / 40.430 | 1.191 / 8.274 / 40.361 |
| exactly-coincident consecutive vertices | 33 | 33 |
| distinct vertex pairs closer than 0.5 m | 1197 | 1167 |
| median separation of those pairs (m) | 0.3411 | 0.3434 |
| segments shorter than 1 m | 1533 | 1502 |
| length change, segments over 1 m | — | max 8.3%, mean 1.0% |
| shared ids / shared coordinates with the source | — | 0 / 0 |

The irregular spacing, the short-segment population and the near-coincident
fork endpoints all come through; what changes is the value of every
coordinate. The 8.3% worst-case length change is jitter acting on a
short segment — two endpoints each displaced by ~2 cm move a 1 m segment by
that much — and it is bounded by the same σ everywhere.

**What this still does not achieve.** The network's layout survives, and it
has to: a perturbation large enough to change which road meets which would
destroy the thing the fixture is for. Someone holding the source could
recognise the layout. What they could not do is recover a coordinate.

### What it is measured to contain

Counted with `lanelet2` and a vehicle routing graph over each map, so the
numbers mean the same thing in every column.

Counted on the committed fixture, after the perturbation. Every topological
entry is unchanged from the un-perturbed rebuild, as expected: the
perturbation moves coordinates, not primitives, and never splits a shared
vertex. The vertex-gap row is the one that moves, and only in the third
significant figure — before the perturbation it read 1.35 / 8.75 / 40.44.

| | `junction_cluster` | `nishishinjuku` | `junction_fanout_mini` |
|---|---|---|---|
| lanelets | 190 | 979 | 9 |
| connectors (`turn_direction`) | 122 | 387 | 3 |
| **forking connectors** (>1 successor) | **12** | 3 | 0 |
| chained connectors (connector → connector) | 31 | 21 | 0 |
| merge points (>1 predecessor) | 22 | 132 | 0 |
| lanelets with a lateral neighbour | 41 | 285 | 0 |
| non-vehicle lanelets | 19 | 95 | 0 |
| regulatory elements | 95 | 732 | 0 |
| boundary vertex gap, median / p95 / max (m) | 1.36 / 8.73 / 40.37 | 2.85 / 6.04 / 122.85 | 20.00 / 20.88 / 20.88 |

Converted it yields 137 roads, 5 junctions and 100 connecting roads, against
789 / 74 / 487 for Nishi-Shinjuku and 5 / 1 / 2 for `junction_fanout_mini`.
Conversion takes about 33 s including the post-conversion ASAM analysis the
CLI always runs.

**What it adds over `nishishinjuku`.** Forking connectors, at 12 in 190
lanelets against 3 in 979 — a density about twenty times higher, in a fraction
of the conversion time (about 40 s against about two minutes).

**What it does not add.** Irregular boundary digitisation is *not* exclusive
to it: Nishi-Shinjuku is a real map too and its worst vertex gap is three
times larger. Against the `*_mini` fixtures both real-derived maps supply that
property; against each other they do not differ in it.
