# One report, carried through

The map here is `nishishinjuku.osm`, the public test map, so its ids can be
shown. Read every id below as something that, on a private map, exists only
in your terminal and under `private/`.

Measured on `integration/2026-11-09` at `3386cdd7`, 2026-11-12, in the
project container, with `map=junction_cluster`.

This example is kept because of how it turned out: the finding at the place
was real output of a real instrument, it matched the symptom, it reduced and
rebuilt cleanly -- and **it was not a defect of the converter**. Step 3 is
where that was caught, and the report changed subject there.

## The report

> On this map, around lanelet 3002118, a route planned through the junction
> stops: it cannot continue into the lane it should.
> The owner agreed to a reproducer.

Map: yes. Place: a lanelet id. Symptom: a route that cannot continue, seen
when following the route on the converted file.

The two questions, asked before anything else: may counts measured on the
map be published -- yes (it is the public test map); may a rebuilt excerpt
be published -- yes, the reporter said so.

## 1. Fingerprint

```
$ python scripts/map_issue.py fingerprint nishishinjuku.osm --out OUT -- map=junction_cluster
conversion: completed in 132 s
  qc checkers: {'total': 26, 'completed': 25, 'skipped': 1, 'errored': 0, 'errors_matching_the_ignore_list': 0}
  qc:asam.net:xodr:1.7.0:lane_smoothness.contact_point_no_horizontal_gaps  x1  (1 located)
  route:missing_succession  x9  (9 located)
```

How it was read: the conversion completed, so nothing is a crash. 25 of the
26 qc checkers completed; the one skipped is a rule for OpenDRIVE 1.8 and
the file declares 1.7, so "1 qc error" speaks for everything that applies.
Two kinds of finding, both located. This map's preset is the neutral one
already, so one fingerprint was enough.

The first time this was run it read `'completed': 24, ..., 'errored': 1`,
and the note above said "a lower bound". The checker that errored was fixed
in the revision `pyproject.toml` pins; the container's environment still
held the previous one. If a checker errors, look at
`asam_qc_opendrive-*.dist-info/direct_url.json` in the environment before
writing the tally down: after the pin moves, the first run measures the old
checker (`uv sync --frozen --dev`, then read the second run).

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
route that cannot continue. A candidate. Not yet an explanation.

## 3. Is it the output that is wrong, or the instrument?

The instrument says lanelet 3002118 cannot be followed into lanelet 425. A
second reading that does not go through the instrument: which roads were
those two lanelets emitted on?

```
3002118 -> 425   source slots [(647, 1), (648, 1)]   target slots [(647, 1)]   shared [(647, 1)]
3002118 -> 526   source slots [(647, 1), (648, 1)]   target slots [(648, 1)]   shared [(648, 1)]
```

(read from `lanelet_to_road_and_lane` and `lanelet_additional_road_lanes` in
the mapping sidecar.) Both lanelets of each pair sit on the **same road and
lane**: the succession happens inside one road and needs no link at all. The
output is right. The instrument counted a succession between the lanelet's
*other* copy and the target, which no file would state.

All nine instances on the map have that shape. Run with the corrected
instrument, the same conversion reports 0 missing successions and 9 internal
ones.

So the report changed subject. Not "the converter drops a succession", but
"`route-fidelity` reports a succession as missing when the two lanelets
share one of several roads" -- and the reporter's symptom is explained only
if they followed the route the way the instrument does (through the first
slot of each lanelet). That went into the issue as an open question to
them, not as a conclusion.

Had this step been skipped, everything below would still have worked, and
the issue would have asked a maintainer to fix a defect that does not exist.

## 4. Reduce

The same machinery reduces a false positive as well as a defect; the
reproducer is now a test case for the instrument.

```
$ python scripts/map_issue.py reduce nishishinjuku.osm --from OUT \
      --finding route:missing_succession --place --timeout 300 -- map=junction_cluster
reduce: starting from 5 lanelets
#1: 5 lanelets -> interesting (exit 0, 13.0 s) [eval-0001]
...
5 -> 4 lanelets in 18 evaluations, 91 s
wrote OUT/private/reduced.osm  (PRIVATE: real ids and coordinates)
```

One round of neighbours (5 lanelets) already showed the finding, and four
of them are needed. 979 lanelets became 4.

## 5. Rebuild

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

The strongest level kept the finding, so it does not depend on
centimetre-scale geometry, and the reproducer carries no surveyed
coordinate.

## 6. Draft

```
$ python scripts/map_issue.py draft --from OUT --figures-agreed \
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

How it was read:

- The place had three instances and the reproducer has two: **a reproducer
  shows the finding, not necessarily every instance of it.** The issue says
  "x2 on the reproducer, x9 on the map".
- `xml.valid_schema` is **new**: the full map does not show it. A cut can
  introduce a finding of its own. It went into the issue as a note
  ("appears on the reproducer only; may be an effect of the cut"), not as
  part of the report.
- The draft is a starting point. The paragraph from step 3 -- what is
  actually wrong, how it was established, and what is still an open question
  for the reporter -- was written by hand under "Expected / Actual", and the
  leak check was run again afterwards.

Then: a search for an existing issue, the whole body shown to the reporter,
and `gh issue create` only after they said yes.

## If the owner had not agreed

**Not to a reproducer:** steps 1 to 3 are the same. Steps 4 and 5 are
skipped: nothing is reduced, no `repro.osm` exists. `draft` writes, under
"Reproducer", `None attached: this report is L0 only`.

**Not to the figures either:** `draft` without `--figures-agreed` writes

```
- **Map**: figures withheld -- the owner has not agreed to publishing measurements of the map
| `route:missing_succession` | withheld | a succession the map states is not traversable in the output |
```

That issue is still actionable: it names the finding, what kind of defect
it is, the converter commit and the kind of place.
