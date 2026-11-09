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
### What the rebuild guarantees, and what is only measured

Three properties are asserted by `test/test_fixture_build_tools.py` on every
run, so they hold for any map this tool is pointed at rather than for one
author's excerpt:

* **The source's MGRS reference does not survive.** `mgrs_code` is dropped.
  On `nishishinjuku.osm` that is 36,936 tags carrying the node's position to
  100 m, which nothing in the converter reads -- `mgrs_code` appears in
  `conversion_config` only as a config key naming a map's grid. Measured on
  the full map: 36,936 in, 0 out, and none of the source's MGRS squares
  appears anywhere in the output.
* **Absolute elevation does not survive.** `ele` is re-origined on the
  excerpt's mean, so every difference between two nodes is preserved exactly
  and the absolute height is not. Measured: the source's `ele` spans
  28.533..48.434 m and the output's spans -9.761..10.164 m, with **zero**
  values in common. It is not dropped, because lanelet2 reads it into
  `point.z` and the elevation profile is built from those -- dropping it
  would flatten the fixture and change what the converter emits.
* **Tag values that are source ids do not survive**, and every tag key that
  occurs in the reference map is classified. The rebuild passes unrecognised
  tags through on purpose, since an allowlist would silently drop a tag the
  converter learns to read later; what makes that default safe is
  `test_every_tag_key_in_the_reference_map_is_classified`, which fails on a
  key nobody has looked at rather than letting it through.

`local_x`/`local_y` are transformed with the coordinates rather than dropped,
for the same reason as `ele`: measured, 81,167..82,280 becomes -543..+568
with zero values in common.

**What this does not achieve.** The network's layout survives, and it has to:
a perturbation large enough to change which road meets which would destroy
the thing the fixture is for. Someone holding the source could recognise the
layout. What they could not do is recover a coordinate.

### What a crop does to the lanelets it keeps

`crop_lanelet2_map.py` is exact about geometry -- kept ways and nodes are
copied verbatim -- and **lossy about regulatory attributes**. A regulatory
element is kept only when every lanelet it names survived the cut; otherwise
it is dropped *and* the `<member role="regulatory_element">` reference is
removed from the lanelets that own it, including ones that were kept.

That is necessary rather than incidental: a `RightOfWay` whose yielding
lanelet was cut away fails lanelet2's own constructor, so keeping it would
make the excerpt unloadable. But it means an excerpt is not simply a subset
of its source. Measured on the reference map: keeping only lanelet 306 of
`right_of_way` element 123826 -- whose yield side is lanelets 309, 310 and
311 -- reports `regulatory_elements_dropped: 1` and strips element 123826
from the retained lanelet 306.

`write_excerpt` returns that count in its census. Anything that iterates
excerpts should report it, because a non-zero value means the behaviour being
measured may have changed for a reason the cut introduced rather than for the
defect under investigation.

### Figures that are not in this file

An earlier version of this document carried measurement tables for one
author's 190-lanelet excerpt: topology counts, boundary-vertex gap
percentiles, conversion timings. They are removed. The fixture is not
committed, the crop box in the procedure above is a placeholder, and the
figures therefore described an artifact no reader could reproduce -- two of
the tables disagreed with each other about the same artifact. What is stated
here instead is either asserted by a test or measured on a map that *is*
committed.
