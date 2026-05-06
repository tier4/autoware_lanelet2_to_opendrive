# Lanelet2 ↔ OpenDRIVE Specification Mapping

Originally a Phase A deliverable of the Lanelet2→OpenDRIVE conversion
completion plan (commit `71a552b`, 2026-04-28). Refreshed 2026-05-06 to
reflect that, in addition to the Phase B P0 / P1 work (P0-1 sidewalk,
P0-2 junction endpoint pinning, P0-3 roadMark, P0-4 lane speed, P1-1
junction priority, P1-2 N:M laneLink, P1-5 asymmetry root cause), the
following also ship on master: bug #291 (diverging-road successor →
junction, #472), header bounds from map extent (#473), traffic-light
arrow `@subtype` bitmask (#474/#475), `<arc>` geometry primitive (#476),
parking lots P2-1 (#448), `<lane><access>` (#477), `<lane><height>` for
sidewalks (#480), and the `lane_id` assignment contract hardening (#481).

The remaining open converter-side items are: `<spiral>` (still a stub),
pedestrian traffic-light emission (#470), highway spline budget tuning
P2-3 (#442), country-aware sign codes P2-4 (#443), and `<laneOffset>`
wiring. None are user-visible on AWSIM Shinjuku.

Read these documents in order:

1. [opendrive-14-carla-profile.md](opendrive-14-carla-profile.md) — the
   OpenDRIVE subset CARLA ue5-dev actually reads.
2. [lanelet2-autoware-profile.md](lanelet2-autoware-profile.md) — the Lanelet2
   data model consumed by this converter (with Autoware extensions),
   grounded in AWSIM 新宿.
3. [mapping-matrix.md](mapping-matrix.md) — element-by-element conversion
   matrix with status labels (FULL / LOSSY / MANUAL / UNSUPPORTED_BY_CARLA /
   MISSING_IN_LANELET2 / NOT_IMPLEMENTED / OUT_OF_SCOPE).
4. [feasibility-verdict.md](feasibility-verdict.md) — conclusion on whether
   full-fidelity conversion is possible, and in which areas.

The implementation roadmap that follows from these documents is tracked
separately in the Phase B plan.
