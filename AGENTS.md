# AGENTS.md

Guidance for Codex agents working on Lanelet2 -> OpenDRIVE converter changes in
this repository.

## Scope

- Treat this repository's converter work as focused on the Lanelet2 -> OpenDRIVE
  pipeline implemented by `autoware_lanelet2_to_opendrive`.
- Keep the existing `autoware_lanelet2_to_opendrive` architecture, data model,
  configuration style, and current behavior as the baseline.
- Do not broaden converter tasks into SUMO/net.xml, TeraSim, standard OSM
  output, visualization tools, CARLA driving tools, or other surrounding
  simulation utilities unless the user explicitly asks for that scope.

## KAIT Reference Work

- Treat the KAIT `lanelet-map-bridge` work as a reference implementation, not as
  code to copy wholesale.
- A KAIT-side patch is not sufficient justification for changing this converter.
  First verify whether the same problem reproduces, or establish a concrete test case demonstrating the missing or incorrect behavior, in the current `autoware_lanelet2_to_opendrive` implementation.
- Distinguish general OpenDRIVE conversion defects from consumer-specific
  workarounds discovered through CARLA, RoadRunner, or another downstream tool.
  Integrate general converter fixes into the model/generator path when possible;
  keep consumer-specific behavior explicit and opt-in.

## Change Discipline

- Prefer small, independent changes. Avoid large refactors, unrelated formatting
  churn, and opportunistic cleanup.
- For bug fixes, follow this order whenever practical:
  1. Reproduce the current behavior.
  2. Add or identify a focused regression test.
  3. Apply the smallest fix consistent with the existing design.
- Prefer fixing root causes in the OpenDRIVE model, road/lane/junction/signal
  construction, or generator logic over XML post-processing.
- Preserve existing public behavior unless the task explicitly calls for a
  behavior change and the regression risk is covered.

## Test Policy

- Use the existing test suite as the regression baseline before adding new
  tests.
- Add tests only where existing coverage does not already protect the behavior
  being changed.
- Do not add XML full-text equality tests or byte-level golden files by default.
  Add such tests only when the task truly requires byte-stable output and the
  reason is documented.
- Prefer semantic OpenDRIVE assertions over formatting assertions. Validate
  meaningful content such as:
  - road and lane structure
  - lane type
  - lane width
  - plan view geometry and coordinate relationships
  - predecessor/successor links
  - lane links
  - junctions and connections
  - road markings
  - signals and controllers
  - objects such as stop lines and crosswalks
- Do not over-constrain values that may legitimately change while preserving
  meaning, including ID allocation, XML element order, formatting, and
  insignificant floating-point representation differences.
- After implementation changes, run the smallest relevant test set first,
  followed by the broader existing regression suite appropriate to the
  affected area.
- Report which tests were run, their results, and any tests that could not be
  executed.

## Validation

- When adding validation, prefer checks that express OpenDRIVE consistency or
  Lanelet2-to-OpenDRIVE semantic fidelity.
- Existing checks such as pytest, QC validation, mapping validation, and targeted
  converter tests should be reused where they fit the change.
- New validation should help diagnose converter correctness, not merely lock in
  one downstream consumer's parser quirks.

## Documentation Boundary

- Keep this file limited to durable development policy.
- Do not record task-specific decisions here, such as how a particular Lanelet2
  subtype should map, which signal country to emit, whether a specific KAIT patch
  should be adopted, or the current integration order. Document those decisions
  in the relevant issue, PR, task note, or feature documentation.

## Relationship To Repository Rules

This file adds converter-specific guidance for Codex agents. It does not replace
repo-wide rules in `CLAUDE.md`, `README.md`, `.pre-commit-config.yaml`,
`pyproject.toml`, and `autoware_lanelet2_to_opendrive/docs/development.md`.
For environment setup, test/lint/format commands, language policy, PR/issue
workflow, git safety, and dependency management, follow those existing rules.
