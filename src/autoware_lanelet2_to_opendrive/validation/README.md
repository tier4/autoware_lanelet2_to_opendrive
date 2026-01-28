# OpenDRIVE Validation Tools

This directory contains validation and diagnostic tools for OpenDRIVE files generated from Lanelet2 maps.

## Available Tools

### 1. validate-opendrive

Validates OpenDRIVE file structure and lane link consistency.

**Usage:**
```bash
# Basic validation
uv run validate-opendrive <path_to_xodr_file>

# Verbose output
uv run validate-opendrive -v <path_to_xodr_file>
```

**Checks performed:**
- Lane links within roads (section-to-section connections)
- Lane links between roads (road-to-road connections)
- Bidirectional link consistency
- XML structure validity

**Example:**
```bash
uv run validate-opendrive test/data/lanelet2_map.xodr
```

### 2. diagnose-lane-links

Detailed analysis of lane link issues in OpenDRIVE files.

**Usage:**
```bash
# Analyze a specific road
uv run diagnose-lane-links <xodr_file> --road <road_id>

# Find broken lane links
uv run diagnose-lane-links <xodr_file> --broken

# Check LHT/RHT consistency
uv run diagnose-lane-links <xodr_file> --rule-check
```

**Features:**
- Detailed road structure analysis
- Lane ID and type inspection
- Link predecessor/successor verification
- Traffic rule consistency checking
- Broken link detection

**Examples:**
```bash
# Analyze Road 0 in detail
uv run diagnose-lane-links test/data/lanelet2_map.xodr --road 0

# Find all broken lane links
uv run diagnose-lane-links test/data/lanelet2_map.xodr --broken

# Check traffic rule consistency
uv run diagnose-lane-links test/data/lanelet2_map.xodr --rule-check
```

### 3. debug-lht-links

Debug tool for LHT (Left-Hand Traffic) lane link generation.

**Usage:**
```bash
uv run debug-lht-links
```

**Purpose:**
- Debug lane link generation for LHT roads
- Trace lanelet-to-lane mapping
- Identify self-referencing lane links
- Inspect routing graph connections

**Note:** This tool is designed for development and debugging purposes.

## Common Issues Detected

### Self-Referencing Lane Links

When a lane's predecessor or successor points to itself:

```xml
<lane id="1">
  <link>
    <predecessor id="1" />  <!-- ❌ Self-reference! -->
    <successor id="1" />    <!-- ❌ Self-reference! -->
  </link>
</lane>
```

**Symptom:** CARLA crashes with segmentation fault when loading the map.

**Detection:** Use `diagnose-lane-links --road <road_id>` to inspect specific roads.

### Missing Reverse Links

When lane A links to lane B, but lane B doesn't link back to lane A:

```
Road 0, Lane 1 → Road 214, Lane 1  (forward link exists)
Road 214, Lane 1 ↛ Road 0, Lane 1  (reverse link missing)
```

**Symptom:** May be intentional for one-way roads, but can indicate data issues.

**Detection:** Use `validate-opendrive` which reports bidirectional consistency warnings.

### Broken Lane References

When a lane links to a non-existent lane ID in the target road:

```xml
<lane id="1">
  <link>
    <successor id="99" />  <!-- ❌ Lane 99 doesn't exist in successor road -->
  </link>
</lane>
```

**Symptom:** CARLA may crash or ignore the connection.

**Detection:** Use `diagnose-lane-links --broken` to find all broken references.

### LHT/RHT Inconsistency

When lane placement doesn't match the traffic rule:

```xml
<road rule="LHT">  <!-- Left-Hand Traffic -->
  <lanes>
    <laneSection>
      <right>  <!-- ❌ LHT should use LEFT lanes, not RIGHT -->
        <lane id="-1" />
      </right>
    </laneSection>
  </lanes>
</road>
```

**Symptom:** Incorrect lane positioning and routing.

**Detection:** Use `diagnose-lane-links --rule-check`.

## Integration with Development Workflow

### During Development

Run validation after generating OpenDRIVE files:

```bash
# Generate OpenDRIVE
uv run convert lanelet2_map.osm --output output.xodr

# Validate the output
uv run validate-opendrive output.xodr

# If issues found, diagnose specific roads
uv run diagnose-lane-links output.xodr --road 0
```

### In CI/CD Pipeline

Add validation to your CI/CD pipeline:

```yaml
# Example GitHub Actions workflow
- name: Validate OpenDRIVE output
  run: |
    uv run validate-opendrive test/data/lanelet2_map.xodr
    if [ $? -ne 0 ]; then
      echo "OpenDRIVE validation failed"
      exit 1
    fi
```

### Pre-commit Hook

Add validation to pre-commit hooks:

```yaml
# .pre-commit-config.yaml
repos:
  - repo: local
    hooks:
      - id: validate-opendrive
        name: Validate OpenDRIVE files
        entry: uv run validate-opendrive
        language: system
        files: \.xodr$
```

## Python API Usage

You can also use these tools programmatically:

```python
from autoware_lanelet2_to_opendrive.validation import (
    OpenDriveValidator,
    analyze_specific_road,
    find_broken_links,
    check_lht_vs_rht_consistency,
)

# Validate an OpenDRIVE file
validator = OpenDriveValidator("path/to/file.xodr")
is_valid = validator.validate()
validator.print_report()

# Analyze a specific road
analyze_specific_road("path/to/file.xodr", road_id="0")

# Find broken links
broken = find_broken_links("path/to/file.xodr")
for road_id, lane_id, target_road, target_lane, link_type in broken:
    print(f"Broken {link_type}: Road {road_id}, Lane {lane_id} → "
          f"Road {target_road}, Lane {target_lane}")

# Check traffic rule consistency
issues = check_lht_vs_rht_consistency("path/to/file.xodr")
for issue in issues:
    print(issue)
```

## Troubleshooting

### Command not found

If you get "command not found" errors, ensure the package is installed:

```bash
uv pip install -e .
```

Then use `uv run` prefix:

```bash
uv run validate-opendrive <file>
```

### Import errors

If you get import errors when using the Python API, check your Python path:

```python
import sys
print(sys.path)
```

And ensure the package is installed in your current environment.

## Related Documentation

- [LANE_LINK_BUG_REPORT.md](../../../LANE_LINK_BUG_REPORT.md) - Detailed analysis of lane link issues
- [OpenDRIVE Specification](https://www.asam.net/standards/detail/opendrive/)
- [CARLA OpenDRIVE Documentation](https://carla.readthedocs.io/en/latest/core_map/#opendrive-standalone-mode)

## Contributing

When adding new validation checks:

1. Add the check to the appropriate tool (`validate_opendrive.py` or `diagnose_lane_links.py`)
2. Update this README with documentation
3. Add unit tests in `test/test_validation.py`
4. Update the `__init__.py` to export new functions if needed
