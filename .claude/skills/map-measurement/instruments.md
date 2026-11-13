# Instruments

Every instrument answers one question in one unit and is blind to the rest.
Pick by the question, and say which instrument produced a figure. All of
them run in the project container (`docs/docker.md`).

| question | instrument | counts | blind to |
| --- | --- | --- | --- |
| Will the map load at all? | `map-preflight` | problem classes, with element ids | anything that loads |
| Is the document internally consistent? | structural invariants (`xodr_invariants.check_invariants`, also printed by `analyze` and at the end of `convert`) | violations by code | geometry accuracy; lane links at junction-typed ends |
| Does it conform to ASAM OpenDRIVE? | `qc-validate`, `analyze` | issues by rule, **per checker that completed** | lane widths; whatever a skipped or crashed checker would have found |
| Can you drive what the map says you can? | `route-fidelity` | successions: stated, missing, surplus; dangling lane links | geometry; reachability beyond one transition |
| How far is the reference line from the source? | `reference_match_distances` in `<out>.mapping.json` (written by `convert`) | roads past 0.5 m / 1.0 m, per-road max | lanes other than the reference; elevation |
| What changed structurally between two outputs? | `xodr-digest` | the fields it extracts, per road | every field it does not extract; roads re-identified across a regrouping |
| Does the consumer accept it? | `carla-import-test`, or `carla.Map(name, xodr_text)` in a probe | load, waypoints, junctions | anything CARLA does not read (lane-level `<link>` semantics differ under `rule="LHT"`) |
| Which way does the consumer face on a short connecting road? | `scripts/carla_connector_yaw.py` | driving lanes on roads under 0.5 m, by yaw difference to the road feeding them: within 5 deg, 5 to 80, crosswise, reversed | connectors whose `<predecessor>` is not a road (counted, not compared); roads over the limit |
| Where the two lanelet -> (road, lane) mappings disagree, which one is right? | `scripts/carla_mapping_judge.py` | disagreeing lanelets by which mapping CARLA's answer matches, beside three controls where the mappings agree | inside junctions roads overlap, so CARLA names a neighbour for about one lanelet in eight even where the mappings agree |

`python scripts/map_issue.py fingerprint` runs the crash, invariant, qc and
route instruments in one go and reports codes and counts.

## Commands

```bash
# Pre-flight (exit 1 only when the load will fail)
map-preflight <MAP>.osm

# ASAM bundle + mapping cross-validation + invariants; the long report.
# The second argument is the map the sidecar's ids refer to: the
# .preprocessed.osm when preprocessing ran, otherwise the input.
analyze <OUT>.xodr <MAP or OUT.preprocessed.osm> --max-issues 5

# ASAM bundle as a gate. An empty ignore file shows what the committed
# ignore-list (conf/qc_ignore_patterns.txt) would otherwise hide.
qc-validate <OUT>.xodr
qc-validate <OUT>.xodr --ignore-file <EMPTY_FILE>

# Successions (arguments are positional)
route-fidelity --json <MAP>.osm <OUT>.xodr <OUT>.mapping.json

# Structure digest for a before/after diff
xodr-digest <OUT>.xodr -o <OUT>.digest.json

# CARLA client-side load (needs the `carla` profile's image)
carla-import-test <OUT>.xodr --skip-tm

# CARLA as a third party (same image; `carla.Map()` needs no server).
# Both print counts and shares only -- no id, no coordinate.
python scripts/carla_connector_yaw.py <OUT>.xodr --label <COMMIT>
python scripts/carla_mapping_judge.py <OUT>.xodr <MAP or OUT.preprocessed.osm> --label <COMMIT>
```

What to read from the output, in this order:

```bash
grep -E "Traceback|Checkers :|Issues   :|Structural invariants|FINAL RESULT" <log>
```

1. `Traceback` -- an instrument died; everything after it is partial.
2. `Checkers : 26 total | N completed | N skipped | N errored` -- how much of
   the bundle the next line speaks for.
3. `Issues   :` -- only now, and only as "N with C checkers completed".

## Reading each one correctly

**qc.** Which checker revision ran is part of the basis: read
`asam_qc_opendrive-*.dist-info/direct_url.json` in the environment and
compare it with the revision `pyproject.toml` pins. An environment that was
built before the pin moved keeps the old checker until it is synced, and an
`errored` checker is the usual sign. A schema violation makes every checker
that requires a valid document skip, so one defect can silence most of the bundle and leave "0 errors".
There is no configuration in which all 26 run (declaring 1.7 skips one
1.8-only rule). When a fix lets a checker run that was skipped before, the
total can go *up*; compare per rule, and recompute every share.

**Structural invariants.** Each was at zero on Nishishinjuku when added, so
any violation is a regression or a new map exposing a defect. The lane-link
checks cover road-typed ends only.

**route-fidelity.** A *missing* succession is a loss, reported as a share; a
*dangling* lane link is a file a consumer can crash on, and fails the run. A
succession between two lanelets emitted on the same road is internal and is
not a transition. Synthetic connectors are walked through, not counted.

**Reference-line distance.** Symmetric point-to-segment maximum per road
(reference line sampled along arc length against the source boundary
polyline, both directions). The headline is the count past each threshold
and the worst road -- not the mean, which is diluted by road length, and not
a percentile, which depends on how many easy roads the map has. Past 1.0 m
is not inaccuracy: it is a different road, i.e. a wrong match or relation.

**xodr-digest.** A digest diff is a claim about the extracted fields only.
It keys roads by id, so it is meaningful only between two conversions that
grouped lanelets the same way.

**CARLA.** Execute it; do not reason from the parser's source.
`carla.Map()` runs without a server. Probe at lane ends, not mid-road. A
dangling `<successor>` at a road-typed end segfaults the client (exit 139)
and no gate in this repository catches it.

**Mapping cross-validation.** A failure of this section of `analyze` says
the conversion-time mapping and the geometric re-derivation disagree, not
which of them is wrong. `scripts/carla_mapping_judge.py` asks CARLA which
lane lies under each disagreeing lanelet. Read its three controls first: if
CARLA does not reproduce the answer where the two mappings agree, it cannot
arbitrate where they do not.

## Writing a probe

When no instrument answers the question, write a throwaway script outside
the tracked tree (`_lcprobe/` and `.claude/state/` are ignored). Give it:

- the `.xodr`, the map and the `.mapping.json` as arguments, and a `--label`
  that carries the commit, printed on the first line of its output;
- a count **and** the denominator it is a share of;
- a control it prints beside the result: the same measurement over a
  population the change does not touch, or over the previous output.

Then read [vacuous-checks.md](vacuous-checks.md) and decide what a pass
would look like if the property were false. A probe that keeps earning its
place becomes a console script with tests, as `route-fidelity` did.

To put a probe through the same locate / reduce / report procedure as the
built-in instruments, make it print one JSON object per instance --
`{"code": "...", "message": "...", "roads": [...], "lanelets": [...]}` --
and pass it as `python scripts/map_issue.py fingerprint ... --probe <PROBE>`.
Its findings are `probe:<code>`.
