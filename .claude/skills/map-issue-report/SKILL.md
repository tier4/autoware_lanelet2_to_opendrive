---
name: map-issue-report
description: Turns a report of the form "on this map, around here, I see this defect" into a filed issue without the map leaving its owner -- resolves the place to lanelets, asks the instruments what they see there, adds a probe when none of them sees the symptom, reduces the map to the few lanelets that still show it, rebuilds those as a synthetic map, verifies nothing of the source leaks, and drafts the issue. Use this whenever someone points at a place on a Lanelet2 or converted OpenDRIVE map and says something looks wrong there (a lanelet id, a road id, coordinates, a simulator position), whenever a defect shows on a private or customer map that cannot be shared or uploaded, whenever asked to file or draft an issue about converter output on a specific map, and whenever a real-map defect needs a small reproducer -- even if the request does not mention an issue or a reproducer.
---

# "On this map, around here, I see this"

A report arrives as three things. Get all three before running anything:

| | what to ask for | never goes into the issue |
| --- | --- | --- |
| **the map** | the `.osm`, its projector info, and how the reporter converts it (`map=`, `target=`) | the file, its name, its city or operator |
| **the place** | whatever they were looking at: lanelet ids, a road id or x,y in the `.xodr`, a `carla.Location`, or a latitude and longitude | ids, coordinates, screenshots |
| **the symptom** | what was seen, in which consumer and version, doing what | -- (it goes in, described by *kind* of place: "at a merge", not where) |

The map stays where it is. What leaves is decided by **the owner of the
map** -- not by you, and not by the tool. There are two things to get their
agreement on, and the tool withholds each until you pass its flag:

| ask the reporter | if yes | if no, or they do not know |
| --- | --- | --- |
| "May counts measured on the map be published -- how many lanelets it has, how often each finding occurs?" | `draft --figures-agreed` | the issue names the findings and says the figures are withheld |
| "May a rebuilt excerpt of a few lanelets be published? Its ids and coordinates are new; its layout is real." | `sanitize --owner-agreed` | no reproducer is made; nothing is reduced |

Ask both questions at the start, in those words, and write down the answers.
"The map can't be shared" answers neither: someone who may not share a map
may still be allowed to say it has 36,000 lanelets, or may not. An agreement
given for an earlier report does not carry over.

| level | file | contains |
| --- | --- | --- |
| **L0** | `public/fingerprint.json`, `public/issue.md` | finding codes, scrubbed messages, converter version, the symptom -- and the counts, only with `--figures-agreed` |
| **L1** | `public/repro.osm` | a few lanelets, renumbered, relocated, rotated, perturbed |
| **L2** | `private/reduced.osm` | the same lanelets with real ids and coordinates -- never public |

The tool is `scripts/map_issue.py` in this repository rather than in the
skill folder, because it imports the converter and its instruments and so
only runs in this repository's environment. Run everything in the project
container from the repository root
(`docker compose --profile dev-native run --rm dev-native`, see
`docs/docker.md`), with the map mounted read-only. `--` separates the
converter's Hydra overrides. Keep `OUT` **outside the repository**:
`private/` holds map data, and the commit sequence in `CLAUDE.md` starts
with `git add -A`.

The first time, read [example.md](example.md): one report carried through
every step, with the output each command printed and how it was read -- and
with the finding turning out to be the instrument's mistake, not the
converter's.

## 1. Convert and fingerprint

```bash
map-preflight <MAP>.osm        # a blocking finding is a report to the map's author; stop
python scripts/map_issue.py fingerprint <MAP>.osm --out <OUT>/own     -- map=<their preset> target=<their target>
python scripts/map_issue.py fingerprint <MAP>.osm --out <OUT>/neutral -- map=junction_cluster map.traffic_rule=<LHT|RHT> target=<their target>
```

Read the output in this order: `conversion: DID NOT COMPLETE` (the finding
is `crash:...`); `instrument '<name>' incomplete` and
`qc checkers: {... 'errored': N}` (that instrument's counts are a lower
bound -- say so); then the findings, `instrument:code xN (M located)`.

A preset named after a place or a customer identifies the map as surely as
its file name, so the tool publishes it as `map=<preset>` and the leak check
rejects the name if it is typed back in. A reproducer's command has to be
runnable by someone else, which is one more reason to reduce and rebuild
with the generic `map=junction_cluster`.

Why twice: a preset's preprocessing names lanelet and point ids, and an
excerpt that lacks them cannot be converted with it. Only what **also shows
in `neutral`** can be reduced. A finding that shows only in `own` depends on
the preprocessing or the configured origin: report it at L0 from `own` and
say so.

## 2. Ask the instruments what they see at the place

```bash
python scripts/map_issue.py near <MAP>.osm --from <OUT>/neutral --lanelets <id>,<id>
#   or  --roads <id>,<id>        road ids of the converted file
#   or  --xy=<X>,<Y>             a position in the .xodr's own frame
#   or  --carla-xy=<X>,<Y>       a carla.Location (its y is the .xodr's -y)
#   or  --latlon=<LAT>,<LON>
#   with --radius <metres> (default 15) and --hops <rounds> (default 2)
```

It resolves the place to lanelets, remembers them for step 4, and lists for
**every** finding how many of its located instances are there.

- `no lanelet at that place` -> the frame is wrong (a CARLA position passed
  as `--xy`, a position from a different conversion) or the radius too
  small. Road ids and positions are only valid for the conversion in
  `--from`: they change whenever the set of roads changes.
- **Match the symptom to a finding, do not assume it.** A finding at the
  place is a candidate, not an explanation. Ask whether that kind of defect
  would produce what the reporter saw; if a place has three findings, the
  symptom belongs to at most one. When in doubt, reproduce the symptom by
  *executing* the consumer on the output (`carla.Map()` runs without a
  server) rather than by reading the `.xodr`.
- **Nothing is reported there**, or nothing that would produce the symptom
  -> step 3.

### Before going on: is the output wrong, or the instrument?

A finding at the place that matches the symptom is still only what one
instrument says. The instruments are younger than the converter and have
been wrong before, and everything after this step works just as smoothly on
a false positive as on a defect -- the reduction shrinks it, the rebuild
keeps it, the issue reads well. So check it once, here, by a reading that
does **not** go through that instrument:

- look at the thing itself in the output at the place -- the road and lane
  `<link>`s, the junction's `<connection>` rows, the geometry endpoints --
  or at the mapping sidecar (`lanelet_to_road_and_lane` **and**
  `lanelet_additional_road_lanes`: a lanelet can be emitted on several
  roads), and ask whether what the instrument reports is actually there;
- or execute the consumer on the output and see whether the symptom occurs.

Then count: of the finding's N instances on the map, how many have the same
shape as the one at the place? The issue is about those, not about N.

Your own checking script is an instrument too, and the newest one in the
room. A script that measures the source lanelet's "width" between the end
points of its two bounds will report 22 m for a 3 m lane whose bounds end
at different places, and conclude the map is at fault. Before a script's
verdict decides the outcome, show it one case where you know the answer.

Four outcomes:

- **The output is wrong** -> continue to step 4.
- **The input is unusual, and the output misrepresents it** -> still the
  converter's issue. A lanelet whose bounds differ in length, a point shared
  by an odd number of ways, an elevation step in the source: these are
  things real maps contain. Ask what the source *says* (the shape a reader
  of the Lanelet2 map would draw) and what the output says. If they differ,
  the converter turned a strange input into a wrong output, and "the map is
  like that" is a note for the map's author *in addition to* the issue, not
  instead of it.
- **The instrument is wrong** -> the report changes subject. It is now an
  issue about the instrument, the same steps produce a reproducer for the
  false positive, and whether it explains what the reporter saw becomes a
  question to put to them. [example.md](example.md) is exactly this case.
- **Some instances are one and some the other** -> say which the issue
  covers and how many; the rest are a separate report or not one at all.

Before reducing a private map, also try the public one: convert
`autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm` and see whether
the same finding, of the same shape, shows there. If it does, build the
issue on the public map and the private one never has to be reduced at all.
"The same shape" is a claim to check, not to assume: a fork and a merge
trigger many of the same code paths and not all of them. If the public
reproducer is a fork and the reported place is a merge, say so in the issue.

A reduced excerpt is one more copy of the map, even when it never leaves
`private/`. Do not cut the private map out of curiosity, "so it is ready if
the owner agrees", or to speed up your own investigation when the full
conversion already answers the question. Reduce it when a reproducer was
agreed and the public map does not show the finding.

## 3. No instrument sees it: write a probe

Either the output is right and the consumer is wrong, or no instrument asks
this question yet. Find out by measuring:

1. Reproduce the symptom on the full output, at the place, by executing the
   consumer -- or, when the consumer is not available, by reading from the
   output the quantity the symptom is about (a width, a length, a height).
   If it does not reproduce **at the place**, say so plainly, and then look
   for the same symptom on the whole output: a mistyped coordinate or a
   position from another frame is more common than a phantom. Whatever you
   find elsewhere is a finding in its own right -- check it like any other
   -- and whether it is what the reporter saw is a question for them, not
   something to assume in the issue. Only when nothing like the symptom
   exists anywhere is the report about something else (their consumer
   version, their conversion).
2. Write a **probe** outside the tracked tree: a script run as
   `python PROBE out.xodr map.osm out.mapping.json` that prints one JSON
   object per instance --
   `{"code": "...", "message": "...", "roads": [...], "lanelets": [...]}`.
   `code` names the *kind* of defect; `roads`/`lanelets` say where and stay
   private.
3. **Check the probe before believing it.** A new instrument is wrong more
   often than the converter is, and a probe that fires everywhere or
   nowhere proves nothing about the place. Three checks, all cheap:
   - it reports the place the reporter named (positive control);
   - it stays quiet on a part of the map known to be fine (control group);
   - you know how many instances it finds on the whole map -- one odd
     place and four hundred are different reports, and an implausible
     count is a statement about the probe.
4. Take both fingerprints again with `--probe <PROBE>`, and re-run `near`.
   The finding is `probe:<code>`; pass `--probe` to every later command.
5. Attach the probe to the issue: it is the definition of the defect and
   contains nothing of the map.

## 4. Reduce to the lanelets that still show it

```bash
python scripts/map_issue.py reduce <MAP>.osm --from <OUT>/neutral \
    --finding <instrument:code> --place --timeout 600 -- <the neutral overrides>
```

`--place` starts from the lanelets `near` resolved plus their neighbours (1,
then 2, then 4 rounds) and removes lanelets while the **same finding**
survives. Without `--place` it starts from one instance the fingerprint
located -- use that only when the report named no place. Expect minutes:
each evaluation converts a small map.

- The last line says `N -> K lanelets`. `K` close to `N` means no small
  cause was found; raise `--hops` rather than accept it.
- **Same finding is not same defect.** A code can fire for a reason the cut
  introduced (a regulatory element dropped because its other lanelet was cut
  away, a road end that lost its neighbour). Compare the scrubbed example
  message with the full map's, check the symptom still reproduces on the
  reduced map, and pass `--match '<regex>'` to pin a broad code.
- `no starting neighbourhood shows the finding` -> it needs more context
  than four rounds, or it does not survive without preprocessing. Do not
  widen the predicate to make it pass.

`private/reduced.osm` is L2. It is the map.

## 5. Rebuild it as a synthetic map (L1)

Only with the owner's agreement to publish a reproducer. The command
refuses to run without `--owner-agreed`; pass it when the reporter has told
you the owner agreed, never because an earlier report had it. Without that
agreement skip to step 6: the report is L0.

```bash
python scripts/map_issue.py sanitize --from <OUT>/neutral --owner-agreed \
    --finding <instrument:code> -- <the neutral overrides>
```

It renumbers every element, moves and rotates the excerpt, re-origins
elevation and `local_x`/`local_y`, drops `mgrs_code` and id-bearing tags,
perturbs the coordinates, converts the result and checks the finding is
still there. It keeps the strongest of three levels the finding survives:

| level | warp | jitter | what survives of the geometry |
| --- | --- | --- | --- |
| `perturbed` | 0.25 m | 0.02 m | shape and character; no surveyed coordinate |
| `warped` | 0.25 m | 0 | local shape exactly; distances over ~100 m do not match |
| `rigid` | 0 | 0 | every relative distance exactly; only position and orientation change |

Tell the user which level was needed. `rigid` means the defect lives in
centimetre-scale geometry and the reproducer carries the excerpt's exact
relative shape -- the owner should know that before agreeing.

No level removes the **layout** of those few lanelets: someone holding the
source map could recognise it, though nobody could recover a coordinate. If
the finding survives no level, it depends on something the rebuild changes;
that is a result -- report at L0 and say so.

## 6. Draft, check for leaks, review with the user, file

```bash
python scripts/map_issue.py draft --from <OUT>/neutral [--figures-agreed] \
    --symptom "<what was seen, at what kind of place>" --seen-in "<consumer and version>"
python scripts/map_issue.py leak-check --from <OUT>/neutral     # again after any edit under public/
```

`draft` writes `public/issue.md` in the shape of
`.github/ISSUE_TEMPLATE/map_defect_report.md`. Without `--figures-agreed`
the table shows each finding with its count withheld and the map's size is
not stated; the terminal still shows you the counts (and
`private/fingerprint.json` keeps them), so you can reason with them -- just
do not type them into the issue by hand. `sanitize` and `draft` run
the leak check and remove their output when it fails. It inspects the files
themselves: ids are a fresh counter, no tag carries a source id or position,
elevation and local coordinates are re-origined, no node keeps its latitude
or longitude, the reproducer lies outside the source map's bounding box, and
no text contains a source path, hash, element id or coordinate.

A `LEAK` line is never a false positive to argue with: fix the cause
(`--to-lat/--to-lon` for an overlapping placement) or drop the file. The
check does not read prose for meaning -- **you** keep out of the issue the
map's name, its city or operator, road and intersection names, and
screenshots of the source map. The reporter's own words are where a place
slips in; rewrite "at the exit of X" as "at a motorway exit".

Then:

1. Fill in "Expected / Actual". Mark what you ran as verified and what you
   concluded as inferred.
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

   and attach `public/repro.osm` if, and only if, L1 was agreed -- plus the
   probe, if one was written.

## When the maintainer fixes it

Only the reporter can confirm it on the real map. After the fix, re-run
steps 1 and 2 on the fixed converter and report the new count for that
finding at that place -- again as a count -- and whether the symptom is
gone in the consumer.

## What this procedure cannot do

- It does not make a map publishable. It makes a *defect* reportable.
- A defect that needs hundreds of lanelets of context has no small
  reproducer; L0 is the report.
- A fingerprint from a converter with uncommitted changes
  (`"uncommitted_changes": true`) does not identify what was run.
