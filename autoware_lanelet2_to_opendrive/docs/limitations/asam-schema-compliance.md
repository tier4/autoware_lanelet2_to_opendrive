# ASAM OpenDRIVE Schema Compliance

## Summary

`target=default` output **conforms** to the ASAM OpenDRIVE 1.7 schema: the
asam-qc-opendrive `valid_schema` checker reports zero issues, so all of the
bundle's semantic, geometry, performance and smoothness checkers actually run.

`target=carla` output does **not** conform. It differs from the conforming
output in exactly one construct — the stop-line stencil
`<object type="-1" name="Stencil_STOP">` — which CARLA's own maps use and
which is not a member of the ASAM `e_objectType` enumeration. This is a
deliberate, documented dialect; see
[CARLA dialect deltas](#carla-dialect-deltas) below.

## Why the declared version is 1.7

The `<header>` declares `revMajor="1" revMinor="7"`. Earlier releases declared
1.4, which produced thousands of schema errors for constructs that are
perfectly legal in 1.7:

| Construct | Legal in 1.4? | Legal in 1.7? |
| --- | --- | --- |
| `<road rule="RHT\|LHT">` (`e_trafficRule`) | no | yes |
| `<signal><positionInertial>` | no | yes (deprecated in 1.8) |
| `<lane><access restriction="passengerCar\|bus\|taxi\|…">` | no (1.4 allows only `simulator`/`autonomous traffic`/`pedestrian`/`none`) | yes |

1.7 is also the highest declaration that keeps the whole checker bundle
applicable. asam-qc-opendrive gates each checker on the "definition setting"
embedded in its rule UID (`asam.net:xodr:<definition-setting>:<rule>`) and
requires `declared_version >= definition_setting`:

| Definition setting | Checkers | Runs at 1.4 | Runs at 1.7 | Runs at 1.8 |
| --- | ---: | --- | --- | --- |
| 1.0.0 (basic + schema) | 5 | yes | yes | yes |
| 1.4.0 | 6 | yes | yes | yes |
| 1.7.0 | 14 | no | yes | yes |
| 1.8.0 (`junctions.connection.one_link_to_incoming`) | 1 | no | no | yes |

Declaring 1.8 would additionally *lose*
`junctions.connection.one_connection_element`, which carries
`APPLICABLE_VERSION = "<=1.7.0"`, and would switch the validator to the XSD 1.1
schema in which `<positionInertial>` is deprecated. 1.7 therefore admits 25 of
26 checkers; only the 1.8.0-only `one_link_to_incoming` stays version-skipped.

Raising the declaration is safe for CARLA: `OpenDriveParser::Load` never reads
`@revMajor`/`@revMinor`
([`OpenDriveParser.cpp`](https://github.com/carla-simulator/carla/blob/0.10.0/LibCarla/source/carla/opendrive/OpenDriveParser.cpp),
tag 0.10.0 — the version pinned in `carla_wheels/`). `carla-import-test`
produces byte-identical results for the 1.4- and 1.7-declared files
(40385 waypoints, 1115 topology edges, 813086-byte cooked binary).

## Why the ignore list is empty

`conf/qc_ignore_patterns.txt` used to suppress the `rule`, `positionInertial`
and `access/@restriction` errors above. Every one of them was a real violation
*of the 1.4 schema we were declaring*, not a false positive, and suppressing
them achieved nothing beyond a green CI line: the bundle evaluates the
`valid_schema` precondition on the **raw** issue list
(`qc_baselib.Result.all_checkers_completed_without_issue` tests
`len(checker.issues) == 0`), so an ignore pattern can never restore a skipped
checker. With the 1.4 declaration, 21 of the 26 checkers had never executed.

The list is now empty. Do not add an entry for an error the converter actually
causes — fix the emitter, or record it here as a known limitation.

## CARLA dialect deltas

These are the differences between `target=default` (conforming) and
`target=carla`, measured on the Nishi-Shinjuku fixture. A CARLA-side adapter
would have to reproduce each one from the conforming file. CARLA's parser
ignores `<userData>`, so none of this can be smuggled into the conforming
output — the adapter must derive everything from standard elements.

| # | Delta | default | carla | Schema-legal? | What a CARLA-side adapter must do |
| --- | --- | ---: | ---: | --- | --- |
| 1 | Stop-line object encoding: `<object type="roadMark" name="stop_line_<ls-id>">` vs `<object type="-1" name="Stencil_STOP">` | 196 | 196 | default yes, carla **no** | Rewrite each `<object type="roadMark" name="stop_line_*">` to `type="-1" name="Stencil_STOP"`. CARLA's `ObjectParser` matches on `name.find("Stencil_STOP")` only — `type` is compared against the literal `"crosswalk"` and nothing else — so the `type` rewrite is cosmetic and the `name` rewrite is what matters. CARLA turns each match into `AddSignal(road, id, s, t, name, "no", orientation, zOffset, "OpenDRIVE", "206", …)`, i.e. a STOP-sign signal, not a surface decal. |
| 2 | Stop-line object geometry: `@width` (thickness along the road) | `0.1` m | `2.0` m | both yes | Set `@width` to the stencil size CARLA should render (`stopline.width` in the target config). |
| 3 | Stop-line object `@zOffset` and `@orientation` | measured z, `"none"` | forced `0.0`, `"-"` | both yes | Force `zOffset="0.0"` and `orientation="-"` on the rewritten objects. |
| 4 | `<signal country="DE" type="294" subtype="-1">` (stop-line signal) | 192 | 30 | both yes | Drop the stop-line signals whose linestring already has a `Stencil_STOP` object, otherwise CARLA sees the stop twice (this is what `carla_stop_line=true` suppresses at source today). |
| 5 | `<signal country="DE" type="1000001">` (traffic light) | 172 | 160 | both yes | Apply `exclude_non_junction_signals`: drop traffic-light signals not attached to a junction. |
| 6 | `<signal>` total (incl. `<signalReference>` mirrors) | 508 | 334 | both yes | Follows from #4 and #5. |
| 7 | `<signal><dependency>` | 592 | 30 | both yes | Follows from #4: the stop-line ↔ traffic-light dependency links are only emitted for signals that survive. |
| 8 | `<signal><positionInertial>` | 282 | 270 | both yes (1.7) | Count follows from #5 — the element itself is emitted for **both** targets and is not a CARLA-only construct. |
| 9 | `positionInertial` `@x/@y/@z` values | centroid | centroid shifted by the `traffic_light` offset `(0.0, -5.5, 5.5)`, rotated by `@hdg` | both yes | Apply the same offset so the spawned traffic-light actor sits over the road rather than at the stop line. |
| 10 | `<controller>` / `<control>` | 324 / 282 | 304 / 270 | both yes | Follows from #5. |
| 11 | `<lane rule="RHT\|LHT">` | absent | absent | n/a | **No longer a delta.** It used to be emitted for both targets and is not an attribute of `<lane>` in any OpenDRIVE XSD. The traffic rule is on `<road rule>` for both targets, which is where CARLA reads it; CARLA's `LaneParser` reads only `<lane>` `@id`, `@type` and `@level`. |
| 12 | `<header revMinor>` | `7` | `7` | both yes | **No longer a delta.** CARLA does not read the version. |

The list is complete for this fixture by construction: it comes from an
element-by-element and attribute-by-attribute census of both outputs, plus a
read of `conf/target/carla.yaml` against `conf/target/default.yaml` (the only
differing keys are `exclude_non_junction_signals`, `stopline.width`,
`stopline.carla_stop_line` and the `traffic_light` offsets — `parampoly3.*`,
`width_estimation.*` and `signal.country` are identical). A map exercising
constructs absent from Nishi-Shinjuku (e.g. parking lots) could add rows, but
no *config* difference beyond the four above exists.

Only delta #1 is schema-illegal. Deltas #2–#10 are ordinary
attribute/element-count differences that a conforming file can already express,
so an adapter can produce the CARLA dialect from the `target=default` output
without any non-standard input.

## Known non-conformances that remain

The schema check passes, but the semantic and geometry checkers it was
blocking now run and report findings. These are tracked separately and are not
schema issues; see the converter's `analyze` output for the current counts.

---

[← Back to Limitations Overview](index.md)
