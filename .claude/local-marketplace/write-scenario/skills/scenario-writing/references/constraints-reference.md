# Sweeper Constraint and Binding Reference

## Discovery: Always Read Source First

**Do not rely on the examples below as the definitive list.** Always dynamically discover available types before writing constraints:

1. **Constraints:** Read `autoware_carla_scenario/src/autoware_carla_scenario/sweeper/constraints.py`
   - Search for all `BaseConstraint` subclasses
   - Read the `build_constraint()` factory function to see the `type:` string → class mapping
   - Read constructor signatures for YAML parameters
2. **Bindings:** Read `autoware_carla_scenario/src/autoware_carla_scenario/sweeper/bindings.py`
   - Search for all `BaseBinding` subclasses
   - Read the `build_binding()` factory function to see the `type:` string → class mapping

## Implementing New Constraints/Bindings

When existing types are insufficient for the scenario's sweep requirements:

1. Propose the new type to the user with rationale
2. Wait for approval before implementing
3. For Constraints: inherit from `BaseConstraint`, implement the check interface
4. For Bindings: inherit from `BaseBinding`, implement the resolve interface
5. Register the new type string in the `build_constraint()`/`build_binding()` factory
6. Export from the appropriate `__init__.py`

## Binding Configuration

Bindings auto-derive parameter values from the matched lanelet. Defined in `sweeper/bindings.py`.

Example pattern (discover actual types from source):
```yaml
bindings:
  ego.spawn_s:
    type: stop_line_offset   # Verify this type exists in build_binding()
    offset: 10.0             # Check constructor signature for parameters
```

## Complete Example

> **Note:** The type strings used below (e.g., `and`, `previous_of`, `has_traffic_light_stop_line`) are examples. Always verify these exist in the `build_constraint()` factory before using them.

Traffic light compliance scenario with constraints and bindings:

```yaml
sweep:
  constraints:
    ego.spawn_lanelet_id:
      - type: and
        constraints:
          - type: previous_of
            constraints:
              - type: has_traffic_light_stop_line
              - type: is_junction
          - type: not
            constraint:
              type: in_set
              values: ${map.no_3d_model_lanelet_ids}
          - type: or
            constraints:
              - type: equals
                value: any

  bindings:
    ego.spawn_s:
      type: stop_line_offset
      offset: 10.0
```

**Pattern notes:**
- The `not in_set` constraint excludes lanelets without 3D models (prevents spawn failures)
- The `or equals any` at the end is a debugging placeholder — replace `any` with specific IDs from successful multirun results to narrow down issues
- `stop_line_offset` binding automatically computes the spawn arc-length relative to the stop line

## Constraint YAML Structure

Constraints are nested under `sweep.constraints.<parameter_key>`:
```yaml
sweep:
  constraints:
    ego.spawn_lanelet_id:   # Parameter to constrain
      - <constraint>        # List of constraints (implicitly AND-ed)
```

The parameter key (e.g., `ego.spawn_lanelet_id`) tells the sweeper which config field to populate with the matching lanelet IDs.
