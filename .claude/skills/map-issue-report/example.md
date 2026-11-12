# One report, carried through

The map here is `nishishinjuku.osm`, the public test map, so its ids can be
shown. Read every id below as something that, on a private map, exists only
in your terminal and under `private/`.

Measured on `integration/2026-11-09` at `3386cdd7`, 2026-11-12, in the
project container, with `map=junction_cluster`.

## The report

> On this map, around lanelet 3002118, a route planned through the junction
> stops: it cannot continue into the lane it should.
> The owner agreed to a reproducer.

Map: yes. Place: a lanelet id. Symptom: a route that cannot continue, seen
when planning. Level: L1 was agreed.

## 1. Fingerprint

```
$ python scripts/map_issue.py fingerprint nishishinjuku.osm --out OUT -- map=junction_cluster
conversion: completed in 132 s
  qc checkers: {'total': 26, 'completed': 24, 'skipped': 1, 'errored': 1, 'errors_matching_the_ignore_list': 0}
  qc:asam.net:xodr:1.7.0:lane_smoothness.contact_point_no_horizontal_gaps  x1  (1 located)
  route:missing_succession  x9  (9 located)
```

How it was read: the conversion completed, so nothing is a crash. One qc
checker **errored**, so "1 qc error" is a lower bound and the issue has to
say so. Two kinds of finding, both located. This map's preset is the neutral
one already, so one fingerprint was enough.

## 2. What do the instruments see at the place?

```
$ python scripts/map_issue.py near nishishinjuku.osm --from OUT --lanelets 3002118
the place is 1 lanelets: [3002118]
with 2 rounds of neighbours: 12 lanelets
  qc:...contact_point_no_horizontal_gaps: 0 of 1 located instances in this area
  route:missing_succession: 3 of 9 located instances in this area
```

How it was read: the qc finding is somewhere else on the map -- not this
report. `route:missing_succession` has three instances here, and "a
succession the map states is not traversable in the output" is exactly a
route that cannot continue. The symptom and the finding are the same kind of
thing, so no probe is needed. Had the symptom been "the lane is 30 m wide
here", three missing successions would have explained nothing, and the next
step would have been a probe.

## 3. Reduce

```
$ python scripts/map_issue.py reduce nishishinjuku.osm --from OUT \
      --finding route:missing_succession --place --timeout 300 -- map=junction_cluster
reduce: starting from 5 lanelets
#1: 5 lanelets -> interesting (exit 0, 13.0 s) [eval-0001]
...
5 -> 4 lanelets in 18 evaluations, 91 s
wrote OUT/private/reduced.osm  (PRIVATE: real ids and coordinates)
```

How it was read: one round of neighbours (5 lanelets) already showed the
finding, and four of them are needed. 979 lanelets became 4.

## 4. Rebuild

Without the flag the command refuses, which is the point:

```
$ python scripts/map_issue.py sanitize --from OUT --finding route:missing_succession -- map=junction_cluster
sanitize: a reproducer carries the layout of a few real lanelets. Produce one only when
the owner of the map has agreed to publish it, and then pass --owner-agreed. ...
```

The reporter said the owner agreed, so:

```
$ python scripts/map_issue.py sanitize --from OUT --finding route:missing_succession \
      --owner-agreed -- map=junction_cluster
sanitize: level 'perturbed' (warp 0.25 m, jitter 0.02 m): finding survives
  ok    only expected files: ['fingerprint.json', 'repro.json', 'repro.osm']
  ok    ids are a fresh counter: 168 elements numbered 1..168
  ok    no tag carries a source id or position: none
  ok    elevation is re-origined: mean ele +0.004 m over 153 nodes
  ok    local_x/local_y are re-origined: 153 nodes, mean (-0.016, -0.202)
  ok    no node keeps its latitude or longitude: 0 of 153 nodes unchanged
  ok    placed outside the source map's bounding box: disjoint
  ok    fingerprint.json: no source path, hash, id or coordinate: clean
  ok    repro.json: no source path, hash, id or coordinate: clean
leak-check: PASS
```

How it was read: the strongest level kept the finding, so the defect is
topological rather than a matter of centimetres, and the reproducer carries
no surveyed coordinate.

## 5. Draft

```
$ python scripts/map_issue.py draft --from OUT \
      --symptom "A route planned through a junction stops: the lane it should continue into cannot be reached in the converted map." \
      --seen-in "route-fidelity on the converted .xodr"
...
leak-check: PASS
wrote OUT/public/issue.md
```

The symptom names a *kind* of place ("a junction"), not lanelet 3002118.
The draft's last section reported, for the reproducer:

```
Findings on the reproducer:

- `qc:asam.net:xodr:1.0.0:xml.valid_schema` x1
- `route:missing_succession` x2
```

How it was read -- and this is the step that is easy to skip:

- `route:missing_succession` is there. Good. But the place had three
  instances and the reproducer has two: **the reproducer shows the finding,
  not necessarily every instance of it.** The issue says "x2 on the
  reproducer, x9 on the map".
- `xml.valid_schema` is **new**: the full map does not show it. A cut can
  introduce a finding of its own. It went into the issue as a note
  ("appears on the reproducer only; may be an effect of the cut"), not as
  part of the defect.

Then: "Expected / Actual" filled in by hand, a search for an existing issue
with `route:missing_succession`, the whole body shown to the reporter, and
`gh issue create` only after they said yes.

## If the owner had not agreed

Steps 1 and 2 are the same. Steps 3 and 4 are skipped: nothing is reduced,
no `repro.osm` exists. `draft` writes the same fingerprint table and, under
"Reproducer", `None attached: this report is L0 only`. That issue is still
actionable: it names the finding, its count, the converter commit and the
kind of place.
