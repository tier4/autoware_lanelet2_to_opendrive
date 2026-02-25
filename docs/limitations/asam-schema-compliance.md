# ASAM OpenDRIVE Schema Compliance

## Issue

The generated `.xodr` files do not fully pass the ASAM OpenDRIVE 1.4 schema validator.
Running the built-in `analyze` command will report schema errors related to the `rule` attribute
on `<road>` elements, even though the output is intentionally structured this way.

```
Element 'road', attribute 'rule': The attribute 'rule' is not allowed.
```

## Cause

### CARLA's OpenDRIVE 1.4 Base with `rule` Attribute Extension

This converter targets **CARLA**, which is based on the **OpenDRIVE 1.4** standard.
However, CARLA requires a way to express **left-hand traffic (LHT)** — road networks where
vehicles drive on the left side of the road (e.g., Japan, UK, Australia).

The `rule` attribute (with values `RHT` for right-hand traffic and `LHT` for left-hand traffic)
was officially introduced in **OpenDRIVE 1.7**, not 1.4. CARLA adopted this attribute as
an extension to its 1.4-based implementation to support LHT maps.

As a result, the converter:

1. Declares the output file as OpenDRIVE **`revMajor="1" revMinor="4"`** for CARLA compatibility
2. Emits `rule="LHT"` or `rule="RHT"` on `<road>` elements, which CARLA requires for correct
   traffic direction handling

When the ASAM QC checker validates the file against the declared 1.4 schema, it correctly
flags `rule` as an unknown attribute — because `rule` was not part of the 1.4 specification.
This is a **known false positive** inherent to CARLA's hybrid 1.4+extension format.

## Impact

- **CARLA simulation**: No impact. CARLA reads `rule` correctly and simulates left-hand/
  right-hand traffic as intended.
- **ASAM QC validation**: Reports errors for every `<road>` element containing `rule`
  (typically hundreds to thousands of errors for a real map).
- **Other OpenDRIVE consumers**: Tools that strictly enforce the 1.4 schema will reject
  the `rule` attribute. Tools that support 1.7 or later will accept it.

## Mitigation

The built-in `analyze` command automatically suppresses this known false positive.
The `rule` attribute errors are excluded from the report by default:

```bash
# Default: rule attribute errors suppressed (723 ignored in a typical map)
uv run analyze output.xodr

# Output example:
# Issues : 149 total | 149 errors | 0 warnings | 0 info | 723 ignored
```

To see all issues including the suppressed ones, use `--no-default-ignores`:

```bash
uv run analyze output.xodr --no-default-ignores
```

## Why Not Upgrade to OpenDRIVE 1.7?

Declaring `revMinor="7"` would resolve the schema mismatch, but introduces other risks:

- CARLA's OpenDRIVE parser is based on 1.4 and may behave unexpectedly with a 1.7 declaration
- Additional 1.7 requirements (e.g., mandatory fields, changed element ordering) could
  break compatibility with downstream CARLA tooling
- The converter is specifically designed to produce CARLA-compatible output, and changing
  the declared version would require extensive compatibility testing

Until CARLA officially upgrades its OpenDRIVE support to 1.7, maintaining `revMinor="4"`
with the `rule` extension remains the correct approach for CARLA-targeted maps.

---

[← Back to Limitations Overview](index.md)
