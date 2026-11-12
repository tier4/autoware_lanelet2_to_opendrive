---
name: map-issue-report
description: Procedure for reporting a conversion defect seen on a map that cannot be uploaded -- fingerprint the conversion as codes and counts, locate the finding, reduce the map to the few lanelets that still show it, rebuild those as a synthetic map, verify nothing of the source leaks, and draft the issue. Use when a defect shows on a private or customer map, when asked to file an issue without attaching the map, or when a real-map defect needs a small reproducer.
---

# Reporting a defect without the map

The map stays where it is. What leaves is, at most, three things, and each
step up is a separate decision by **the owner of the map** -- not by you, and
not by the tool:

| level | what it is | contains | who may see it |
| --- | --- | --- | --- |
| **L0** | `public/fingerprint.json` | finding codes, counts, scrubbed messages, lanelet counts by subtype, converter version | public, once the owner agrees that measurement data about the map may be published |
| **L1** | `public/repro.osm` | a few lanelets, renumbered, relocated, rotated, perturbed | public, if the owner agrees to *this file* |
| **L2** | `private/reduced.osm` | the same lanelets with real ids and coordinates | never public; a private channel to a maintainer at most |

Ask the user which level they are authorised for **before producing L1**, and
never infer it from an earlier report. If they do not know, stop at L0.

All commands run in the project container (`docs/docker.md`), from the
repository root. `--` separates the converter's Hydra overrides. Keep `OUT`
**outside the repository**: `private/` holds map data, and the commit
sequence in `CLAUDE.md` starts with `git add -A`.

## 0. Before anything: does the map load?

```bash
map-preflight <MAP>.osm
```

A blocking finding means the map itself is malformed; that is a report to
the map's author, by problem class and count, and nothing below applies.

## 1. Fingerprint the conversion (L0)

Twice -- once as the user converts it, once with a config that names no
preprocessing and no origin:

```bash
python scripts/map_issue.py fingerprint <MAP>.osm --out <OUT>/own     -- map=<their preset> target=<their target>
python scripts/map_issue.py fingerprint <MAP>.osm --out <OUT>/neutral -- map=junction_cluster map.traffic_rule=<LHT|RHT> target=<their target>
```

Read the output in this order:

1. `conversion: DID NOT COMPLETE` -- the finding is `crash:<type>:<hash>`.
2. `instrument '<name>' incomplete` and `qc checkers: {... 'errored': N}` --
   counts from that instrument are a lower bound. Say so in the issue.
3. the findings, `instrument:code xN (M located)`.

Choose **one** finding. One issue per finding.

Why twice: a map preset's preprocessing names lanelet and point ids, and an
excerpt that lacks them cannot be converted with it. Only a finding that
**also shows in `neutral`** can be reduced.

- In both -> continue with `<OUT>/neutral`.
- Only in `own` -> it depends on the preprocessing or the configured origin.
  Report at L0 from `<OUT>/own` and say so (step 5).
- Only in `neutral` -> the preset hides it. Worth its own L0 note; it is not
  the user's defect.

## 2. Reduce to the lanelets that still show it

```bash
python scripts/map_issue.py reduce <MAP>.osm --from <OUT>/neutral \
    --finding <instrument:code> --timeout 600 -- <the neutral overrides>
```

This starts from the lanelets under one instance of the finding plus their
neighbours (1, then 2, then 4 rounds) and removes lanelets while the **same
finding** survives. Expect minutes: each evaluation converts a small map.

Check, do not assume:

- The last line says `N -> K lanelets`. A reduction that ends at `K` close to
  `N` did not find a small cause; raise `--hops` rather than accept it.
- **Same finding is not same defect.** A code can fire for a reason the cut
  introduced (a regulatory element dropped because its other lanelet was cut
  away; a road end that lost its neighbour). Compare the scrubbed example
  message with the one in the full-map fingerprint, and pass
  `--match '<regex>'` to pin it when a code is broad.
- `no starting neighbourhood shows the finding` means it needs more context
  than four rounds of neighbours, or it does not survive without
  preprocessing. Do not widen the predicate to make it pass.

`private/reduced.osm` is L2. It is the map.

## 3. Rebuild it as a synthetic map (L1)

Only with the owner's agreement to publish a reproducer.

```bash
python scripts/map_issue.py sanitize --from <OUT>/neutral \
    --finding <instrument:code> -- <the neutral overrides>
```

It renumbers every element, moves and rotates the excerpt, re-origins
elevation and `local_x`/`local_y`, drops `mgrs_code` and id-bearing tags,
and perturbs the coordinates -- then converts the result and checks the
finding is still there. It tries three levels, strongest first, and keeps
the first on which the finding survives:

| level | warp | jitter | what survives of the geometry |
| --- | --- | --- | --- |
| `perturbed` | 0.25 m | 0.02 m | shape and character; no surveyed coordinate |
| `warped` | 0.25 m | 0 | local shape exactly; distances over ~100 m do not match |
| `rigid` | 0 | 0 | every relative distance exactly; only position and orientation change |

Tell the user which level was needed. `rigid` means the defect lives in
centimetre-scale geometry and the reproducer carries the excerpt's exact
relative shape -- the owner should know that before agreeing.

What no level removes is the **layout** of those few lanelets. Someone who
holds the source map could recognise it. Nobody could recover a coordinate
from it.

If the finding survives no level, it depends on something the rebuild
changes. That is a result: report at L0 and say so.

## 4. Verify nothing leaks

`sanitize` and `draft` run this and refuse to finish when it fails; run it
again by hand after editing anything under `public/`.

```bash
python scripts/map_issue.py leak-check --from <OUT>/neutral
```

It checks the files themselves, not how they were made: ids are a fresh
counter, no tag carries a source id or position, elevation and local
coordinates are re-origined, no node keeps its latitude or longitude, the
reproducer lies outside the source map's bounding box, and no text file
contains a source path, hash, element id or coordinate.

A `LEAK` line is never a false positive to argue with. Fix the cause
(`--to-lat/--to-lon` for an overlapping placement) or drop the file.

The check does not read prose for meaning. **You** keep out of the issue:
the map's name, its city or operator, road or intersection names, and
screenshots of the source map.

## 5. Draft, review with the user, file

```bash
python scripts/map_issue.py draft --from <OUT>/neutral
```

writes `public/issue.md` in the shape of
`.github/ISSUE_TEMPLATE/map_defect_report.md`. Then:

1. Fill in the finding line (if L0 only) and "Expected / Actual". State what
   you ran as verified and what you concluded as inferred.
2. If no reproducer is attached, say which step ended it: needs
   preprocessing, did not survive reduction, did not survive the rebuild.
3. Search for an existing issue with the same finding code.
4. **Show the user the full body and the list of files to attach, and wait
   for an explicit yes.** Publishing cannot be undone.
5. Only then:

```bash
gh issue create --title "[MAP] <instrument:code>: <one line>" \
    --body-file <OUT>/neutral/public/issue.md --label bug
```

   and attach `public/repro.osm` if, and only if, L1 was agreed.

## When the maintainer fixes it

The reporter is the only one who can confirm it on the real map. After the
fix, re-run step 1 on the fixed converter and report the new count for that
finding code -- again as a count.

## What this procedure cannot do

- It does not make a map publishable. It makes a *defect* reportable.
- A defect that needs hundreds of lanelets of context has no small
  reproducer; L0 is the report.
- A fingerprint from a converter with uncommitted changes
  (`"uncommitted_changes": true`) does not identify what was run.
