---
name: map-measurement
description: Measurement-driven procedure for working on the converter -- turn a question about conversion quality into a number with a stated basis, check the instrument before believing it, locate the defect, fix it test-first, and re-measure both states the same way. Use when asked how good the conversion is, when comparing output before and after a change, when investigating a defect seen on a real map, and before quoting any figure about converter output in a PR, issue or report.
---

# Measuring the conversion on a real map

The converter is developed by measuring what it emits for a real map, not by
reading its code and reasoning about what it should emit. This skill is that
loop, and the ways it has gone wrong.

```
question -> instrument -> baseline -> check the instrument -> locate
         -> make it small -> red/green fix -> re-measure both states -> second map -> record
```

Instruments, their commands and what each one cannot see:
[instruments.md](instruments.md). Checks that pass while verifying nothing:
[vacuous-checks.md](vacuous-checks.md). Read the second before writing a
checker, a probe, or the sentence "0 found".

## 1. State the question as a quantity

"Is the junction handling right?" cannot be measured. "How many successions
the Lanelet2 routing graph states are not traversable in the output?" can.
Before running anything, write down:

- **what is counted, and its unit** -- roads, road *ends*, directed links, qc
  issues, sites. `400 roads` and `160 sites` have been the same finding.
- **what it is judged against** -- the source map, the ASAM rules, a consumer
  (`carla.Map()`), or the previous output.
- **what the purpose needs.** The output exists to be a simulator's input for
  testing an Autoware that loaded the original Lanelet2 map. A figure that
  does not bear on that is a curiosity.

## 2. Convert, and keep the evidence of what was converted

Run in the container (`docs/docker.md`); never on the host. Write outputs
**outside the source tree** and record the commit beside them:

```bash
convert map=<preset> target=<target> \
  input_map_path=<MAP>.osm output_map_path=<OUT>/<label>.xodr \
  hydra.run.dir=<OUT>/hydra_<label> > <OUT>/<label>.convert.log 2>&1
echo "exit=$?" >> <OUT>/<label>.convert.log
git rev-parse HEAD > <OUT>/<label>.commit
```

- Always pass `input_map_path` and `hydra.run.dir` (a preset carries no input
  path; without `hydra.run.dir` two runs share `outputs/`).
- A conversion is usable only if the log ends with `exit=0` **and** the file
  closes with `</OpenDRIVE>`. A run that was killed leaves a truncated file
  and, in pytest's temp directory, sometimes a complete one worth reusing.
- The `.xodr`, `.mapping.json` and `.preprocessed.osm` of one run belong
  together (the sidecar pins them by SHA-256). Never mix runs.
- One large-map conversion at a time; cap xdist at `-n 4`. A full-map
  conversion on a city-scale map needs several GB and tens of minutes.

## 3. Check the instrument before you believe the number

Do this *first*, every time, in this order. Each line has cost a withdrawn
figure.

1. **Did it run?** For qc, read the `Checkers :` line before the `Issues :`
   line: a skipped or crashed checker contributes zero issues. Quote "N errors
   with C of 26 checkers completed". An error count is a lower bound.
2. **Can it fail?** Show the instrument a case where the property is false and
   confirm it says so (a positive control). If you cannot construct one, you
   have an identity, not a check.
3. **Is it reading what you changed?** Run it on output your change does not
   touch (a control group). If the "effect" shows there too, the instrument is
   broken, not the converter.
4. **Is the number plausible?** An implausible number is a measurement *of the
   instrument*. Check it against what the format guarantees -- Lanelet2 defines
   lane connectivity by shared points, so a non-zero gap between connected
   lanelets in the *input* cannot be real -- before checking the arithmetic.
5. **Is zero a property of the map?** Nishishinjuku has no lane drop, no
   inverted linestring, no pedestrian signal. A zero there says the structure
   is absent, not that the code is right. Find a map that has the structure.
6. **Is it a minimum over candidates?** Say how many candidates the winner
   beat, and give the single-candidate figure.

If the instrument is wrong, **fix the instrument** and re-measure; do not
reason around it. A fixed instrument changes every figure taken with the old
one -- say which ones.

## 4. Locate the finding in the source map

An output road id means nothing by itself: ids are assigned by enumeration
order and are **reassigned wholesale** when the set of roads changes. Never
match roads of two conversions by id; never carry an id from one output to
another.

Go through the source lanelets instead:

- `<out>.mapping.json` attaches lanelets to roads in **four** fields --
  `lanelet_to_road_and_lane`, `lanelet_additional_road_lanes`,
  `chain_merged_roads`, `exit_fork_duplicate_roads`. Fold all four; reading
  only the first has produced false findings more than once.
- A road with no backing lanelet (`skipped_synthetic_roads`) is a connector
  the converter synthesised; locate it by the roads it links.
- With preprocessing on, the sidecar's ids are those of
  `<out>.preprocessed.osm`, not of the input: a merge replaces two lanelets
  with a new id. "Lanelet 112 does not exist" may mean "it was merged".

`python scripts/map_issue.py fingerprint <MAP> --out <DIR> -- <overrides>`
does this join for every finding and writes it to
`<DIR>/private/locations.json`.

## 5. Make it small, then fix it red/green

A test that converts Nishishinjuku costs minutes per run; a small map costs
seconds. In order of preference:

1. an existing `*_mini.osm` fixture, if the defect does not need surveyed
   geometry (hand-drawn fixtures sit on a 20 m grid and cannot reproduce
   centimetre-scale mismatches);
2. a reduction of the real map:
   `python scripts/map_issue.py reduce <MAP> --from <DIR> --finding <instrument:code> -- <overrides>`
   (delta debugging over lanelets; the predicate asks for the *same* finding,
   not for any failure). Whether the result may be committed is the map
   owner's decision -- see the `map-issue-report` skill;
3. Nishishinjuku itself. Say so in the PR.

Then hand over to the `red-green` skill. The red must fail for the reason the
measurement showed.

## 6. Re-measure both states with one instrument

- **Two code states, two trees.** Clone the repository (`git clone
  --no-hardlinks`), check out the base commit there, give it its own venv
  volume and container, and mount the map read-only into both. Do not flip
  one working tree between branches while something is running in it.
- **One instrument.** Measure both outputs with the *same* tree's tools --
  the newer one. An instrument that changed between the two states measures
  its own change.
- **Label everything with the commit**, in file names and in probe output.
- **Compare parsed values, not bytes.** The header carries a wall-clock date
  and line 2 carries the commit, so a hash only ever says "rebuilt".
  `xodr-digest` normalises both away.
- **Re-measure after every fix**, including the figures you did not expect to
  move. A geometry fix that leaves the reference line unchanged has broken
  junction endpoints before.
- After changing a dependency pin, read the *second* run: the first one still
  tests the old venv.

## 7. Confirm on a second map

A fix measured on one map is a hypothesis about the other. Run the same
instrument on a map with different character (size, traffic rule, presence of
the structure). If you hold no second map, say the result is from one map.

## 8. Record it so it can be checked

Every figure that leaves the session carries its basis:

```
<value> <unit> -- <map and its version>, <branch>@<commit>, <date>,
<instrument and command>, [verified | inferred]
```

- Quote counts and densities across maps, never percentiles (they fall as
  easy roads are added).
- Mark what you ran as **verified** and what you concluded as **inferred**.
  A docstring, a PR body and an old note are inferred until re-run.
- When a later measurement contradicts an earlier figure, **withdraw the
  earlier one in writing** where it was published, with the reason.
- For a private map, publish codes and counts only -- no element id, no
  coordinate, no path -- and only with the owner's agreement. The
  `map-issue-report` skill is the procedure.
- Deferred work gets a written note with the measurement that motivates it,
  not a memory.
