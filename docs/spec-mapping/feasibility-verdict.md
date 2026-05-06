# Feasibility Verdict: Lanelet2 → OpenDRIVE (CARLA ue5-dev)

## Question

Can AWSIM-style Autoware Lanelet2 maps be converted to an OpenDRIVE document
that makes CARLA ue5-dev a full-fidelity consumer — vehicle autopilot and
pedestrian autopilot, respecting traffic control?

## One-line verdict

**Yes for vehicle autopilot, qualified for everything else.** The two map
formats are structurally compatible for urban road networks, provided we
accept a bounded geometric approximation and add a small number of extensions
on both sides. Pedestrian autopilot requires Lanelet2 map work that is
outside this repository.

## Breakdown

### 1. Road geometry

**Feasible with bounded error.** OpenDRIVE encodes a lane as a single
reference line plus a scalar width profile; Lanelet2 encodes it as two
independent polyline bounds. The two representations are not isomorphic,
but any "well-behaved" lanelet (roughly symmetric, monotonic width) can be
fit by a `paramPoly3` reference line plus a `width` polynomial with
average error below 2 m and peak error below 8 m. See
`autoware_lanelet2_to_opendrive.spline.Splines` and
`autoware_lanelet2_to_opendrive.opendrive.geometry.ParamPoly3.from_spline`.

Pathological lanelets (asymmetric, non-monotonic width) used to trigger
`AsymmetryLaneletException` and were skipped. As of #461 (commit `8b2f7f7`,
follow-up to `88c59ab`) that exception is gone: asymmetric lanelets now fit
via the width spline alone, so no automatic skip path remains. Manual
preprocessing via `preprocess_lanelet.py` is still available for shape
clean-up but no longer required to keep a road in the output.

Height (z) is preserved. Banking (superelevation) is discarded; for urban
AWSIM maps this is typically acceptable.

### 2. Road topology and routing

**Fully feasible.** Predecessor/successor chains, junction structure, and
lane-level connectivity map cleanly onto OpenDRIVE. 1:1 and N:M lane links
both ship today (#460, P1-2). Connecting-road endpoints are pinned to the
linked regular roads' rendered endpoints by default (#437/#453/#464,
P0-2), so junction meshes no longer carry the multi-metre gaps that the
spec-mapping originally flagged.

The previous diverging-road defect — road-level `<successor>` pointing at a
single fork instead of a junction — was fixed in #472, closing #291.
Diverging / merging road groups now emit a junction successor with a per-lane
link table, eliminating the unintended lane-change behaviour CARLA's
TrafficManager exhibited.

### 3. Traffic signals

**Feasible today.** Dynamic traffic lights, static stop/yield signs, and
stop-line markings are all emitted and understood by CARLA ue5-dev. The
remaining work is quality of behaviour, not conversion:

- **CARLA TrafficManager does not stop at stop-line signals (type 294).**
  Workaround: emit `dependency` from the traffic light to the stop line so
  NPCs stop at the junction boundary instead.
- **CARLA TrafficManager does not honor `junction/priority`.** Yield rules
  degenerate to "slow to 10 km/h".

Both are CARLA-side issues and out of scope for this converter plan.

### 4. Pedestrians

**Not feasible with AWSIM 新宿 as-is.** CARLA's pedestrian NavMesh is built
from OpenDRIVE `lane type="sidewalk"` chains plus `crosswalk` objects.
AWSIM 新宿 contains 84 crosswalks but only 8 walkway lanelets, and those
walkways are disconnected from the crosswalks. The converter side is now
fully unblocked: P0-1 (#427) emits `lane type="sidewalk"` for walkway
lanelets, and #480 raises sidewalk / shoulder lanes by
`DEFAULT_CONFIG.geometry.sidewalk_height` whenever a curb LineString
bounds them so the kerb step is physically present in the mesh. The
remaining topology gap is on the Lanelet2 authoring side and the NavMesh
remains unusable end-to-end until walkway↔crosswalk connectivity is
fixed there.

This is the biggest "feasibility" caveat in the whole project. Until the
input map is extended, CARLA pedestrian simulation on AWSIM 新宿 will remain
broken regardless of converter quality.

### 5. Static infrastructure (poles, buildings, fences)

**Out of scope for the converter.** Lanelet2 does not model physical street
furniture beyond what is implied by regulatory elements. CARLA obtains these
from its 3D asset pipeline, not from OpenDRIVE.

### 6. Lane-level metadata (road marks, speed limits, access)

**Implemented.** P0-3 (#426) emits `roadMark@type/@color/@weight/@laneChange`
from each LineString's `type`/`subtype`/`color`/`lane_change` attributes.
P0-4 (#428) emits per-lane `<speed>` from the `speed_limit` attribute in
km/h. `<lane><access>` from `participant:*` (#477) and `<lane><height>`
for sidewalks raised above the driving surface (#480) both ship as FULL.

The only remaining lane-level gap is `<laneOffset>` whenever the planView
reference line ever needs to deviate from the driving centerline.
Infrastructure exists in `opendrive/lane_section.py` but no caller
populates it; the field becomes load-bearing only after a multi-section
refactor that is not currently planned.

### 7. Country-specific sign numbering

**Out of scope today.** CARLA hard-codes the German StVO interpretation and
ignores `signal@country`. The converter therefore emits German type codes
even for Japanese signs. For any non-CARLA consumer (SUMO, external viewers)
this becomes important; handled in P2-4.

## Representative information loss

A non-exhaustive list, for planning:

- **Lossy but within tolerance:** road geometry (smoothed; `<line>`,
  `<arc>`, `<paramPoly3>` ship today, `<spiral>` is still a stub),
  elevation (stepped between geometry segments), lane width (polynomial
  fit). Traffic-light arrow info now preserved in `signal@subtype`
  (#474/#475) so this is no longer in the lossy column from the
  converter's side; CARLA itself still ignores `@subtype` at runtime.
- **Fully dropped today, recoverable with implementation work:**
  per-point attributes beyond x/y/z, country-specific sign numbering
  (P2-4 / #443), `<spiral>` curvature transitions (#466 follow-up).
- **Fully dropped, not recoverable:** detection area, no-stopping area,
  no-parking area, rail/tram, buildings, vegetation.
- **Forced by CARLA:** country-specific sign codes (CARLA hard-codes
  German StVO regardless of `signal@country`), stop-line behaviour via
  TrafficManager, junction priority enforcement, pedestrian pathfinding.

## What we have validated

- Vehicle autopilot on 新宿: red-light stop, green-light intersection pass
  (straight / left / right), temporary stop, partial intersection arbitration.
- `analyze_xodr.py` validation against ASAM `qc-framework` rules for the
  emitted `nishishinjuku.xodr`.
- Visual inspection via esmini and the VSCode OpenDRIVE viewer.
- 30+ test files exercising the internal components (see
  `autoware_lanelet2_to_opendrive/test/`).

## What we have not validated

- Full-city conversion on お台場 (runs complete — ~4 hours — but scenarios
  are not yet exercised there).
- External tool compatibility (SUMO, RoadRunner, external qc tools beyond
  the in-repo analyze_xodr).
- Pedestrian scenarios.

## Confidence by area

| Area | Confidence that CARLA ue5-dev will work end-to-end |
|---|---|
| Vehicle routing | High (#291 closed by #472; diverging roads now use junction successors) |
| Traffic lights | High (behavior already proven; arrow info now in `@subtype` via #474/#475) |
| Stop signs | Medium (single verified scenario) |
| Yield signs | Low (CARLA ignores `<junction><priority>`, even though it now ships) |
| Stop-line compliance | Medium (via dependency, not type 294) |
| Road marks (lane-change rules) | High (P0-3 shipped 2026-04 in #426) |
| Pedestrians | Low: `lane[type=sidewalk]` (P0-1, #427) and curb-step `<lane><height>` (#480) now emitted, but Lanelet2 walkway↔crosswalk topology remains broken |
| Highway (お台場) | Medium; `<arc>` (#476) reduces segment-count pressure, P2-3 (#442) still useful for further tuning |

## Recommendation

Phase B P0 (#423) / P1 (#435) trackers and most of P2 are closed.
On top of P0-1..P0-4, P1-1, P1-2 and P1-5 (asymmetry root cause), the
following landed in the 2026-05 wave: #291 (diverging-road successor →
junction, #472), header bounds (B-1 / #473), traffic-light arrow
`@subtype` (B-3 / #474+#475), `<arc>` primitive (B-2 / #476), parking
lots (P2-1 / #448), `<lane><access>` (B-5 / #477), `<lane><height>`
sidewalk curb step (B-6 / #480), and `lane_id` contract hardening
(B-8 / #481).

Remaining converter-side work, in priority order:

1. **Pedestrian traffic-light emission (B-7 / #470).** Code path is
   dormant today (Shinjuku has 0 `pedestrian_traffic_light` REs); needed
   for any non-Shinjuku map that uses them.
2. **Highway spline budget tuning (P2-3 / #442).** `<arc>` already
   reduces the worst-case segment-count blow-up; remaining tuning is
   marginal and Odaiba-specific.
3. **Country-aware sign codes (P2-4 / #443).** SUMO / esmini consumer
   benefit only; CARLA hard-codes German StVO regardless.
4. **`<spiral>` curvature transitions.** The `Spiral` class is a stub
   and the shared evaluation kernel needs extending. #466 follow-up.
5. **`<laneOffset>` wiring.** Infrastructure exists but no caller
   assigns it; only relevant if a future refactor splits the planView
   reference line from the driving centerline.
6. **vectorSignal `userData` (#136).** Now actionable since #474/#475
   produces the data; nice-to-have for RoadRunner round-tripping.

CARLA-side improvements (TrafficManager honouring priority and stop lines,
speed limits) remain a separate track, tracked as context for this plan but
not included as tasks here. The Lanelet2 authoring-side walkway↔crosswalk
topology fix is the only blocker for end-to-end pedestrian scenarios on
Shinjuku and cannot be addressed by the converter alone.
