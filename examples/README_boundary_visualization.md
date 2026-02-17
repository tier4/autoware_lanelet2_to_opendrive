# Boundary Error Visualization Tool

This tool visualizes and analyzes the accuracy of Lanelet2 to OpenDRIVE conversion by comparing original Lanelet2 boundaries with converted OpenDRIVE lane boundaries.

## Purpose

During the Lanelet2 to OpenDRIVE conversion process, geometric transformations (spline fitting, coordinate conversion, etc.) introduce fitting errors. This tool helps you:

1. **Quality assurance**: Validate that conversion accuracy meets acceptable thresholds
2. **Parameter tuning**: Identify which spline parameters need adjustment
3. **Problem diagnosis**: Locate problematic road sections with high errors
4. **Documentation**: Provide visual evidence of conversion quality

## Features

- **Automatic Lane Matching**: Uses 3D nearest-neighbor matching to automatically find corresponding OpenDRIVE lanes without manual ID mapping
- **Visual Error Analysis**: Color-coded boundary visualization (blue=low error, red=high error)
- **Statistical Analysis**: Error distribution histograms with mean, standard deviation, and percentile statistics
- **Multiple Output Formats**: JSON for analysis, PNG for reports, pickle for interactive re-plotting
- **Flexible Configuration**: Adjustable sampling intervals, colormaps, and lanelet filtering

## Installation

The tool is included as part of the `autoware-lanelet2-to-opendrive` package and uses existing project dependencies:

- `pyxodr>=0.1.3` - OpenDRIVE parsing
- `matplotlib>=3.10.7` - Visualization
- `lanelet2>=1.2.2` - Lanelet2 map loading
- `numpy` - Numerical computation
- `mgrs>=1.5.0` - MGRS coordinate projection (optional)

Install the package with:

```bash
uv pip install -e .
```

Or sync dependencies:

```bash
uv sync
```

## Usage

The tool is available as the `visualize` command through `uv run`:

### Basic Usage

```bash
# Visualize all lanelets with default settings
uv run visualize lanelet2_map.xodr lanelet2_map.osm
```

### Specify Lanelet IDs

```bash
# Visualize specific lanelets only
uv run visualize map.xodr map.osm --lanelet-id 120 121 122
```

### Save Output Files

```bash
# Save error data as JSON and visualization as PNG
uv run visualize map.xodr map.osm \
    --output-json errors.json \
    --output-png boundary_comparison.png
```

### Adjust Parameters

```bash
# Use coarser sampling interval and different colormap
uv run visualize map.xodr map.osm \
    --sample-interval 1.0 \
    --colormap viridis
```

### Run Without GUI

```bash
# Save outputs without showing interactive plot
uv run visualize map.xodr map.osm \
    --output-json errors.json \
    --output-png visualization.png \
    --no-show
```

### Specify MGRS Projection

```bash
# Use MGRS code for coordinate projection
uv run visualize map.xodr map.osm \
    --mgrs-code 54SUE
```

## Command-Line Options

### Required Arguments

- `opendrive_file`: Path to OpenDRIVE `.xodr` file
- `lanelet2_file`: Path to Lanelet2 `.osm` file

### Optional Arguments

| Option | Description | Default |
|--------|-------------|---------|
| `--lanelet-id ID [ID ...]` | Specific lanelet IDs to visualize | All lanelets |
| `--boundary {left,right,both}` | Which boundary to compare | `both` |
| `--sample-interval METERS` | S-coordinate sampling interval (m) | `0.5` |
| `--output-json PATH` | Save error data as JSON | Not saved |
| `--output-pickle PATH` | Save matplotlib figure as pickle | Not saved |
| `--output-png PATH` | Save visualization as PNG image | Not saved |
| `--colormap NAME` | Matplotlib colormap name | `coolwarm` |
| `--no-show` | Do not show interactive plot window | Shows plot |
| `--mgrs-code CODE` | MGRS code for Lanelet2 projection | No projection |

## Output Formats

### 1. JSON Output (`--output-json`)

Structured error data for programmatic analysis:

```json
{
  "metadata": {
    "opendrive_file": "path/to/map.xodr",
    "lanelet2_file": "path/to/map.osm",
    "sample_interval": 0.5,
    "timestamp": "2025-02-17T10:30:00"
  },
  "lanelets": {
    "120": {
      "left": {
        "s_coordinates": [0.0, 0.5, 1.0, ...],
        "errors": [0.023, 0.045, 0.031, ...],
        "matched_opendrive_lane_idx": 5,
        "statistics": {
          "min": 0.001,
          "max": 0.234,
          "mean": 0.045,
          "std": 0.032,
          "p95": 0.112
        }
      },
      "right": { ... }
    }
  }
}
```

### 2. PNG Output (`--output-png`)

High-resolution image (300 DPI) with two plots:
- **Left**: Color-coded boundary comparison (blue=low error → red=high error)
- **Right**: Error distribution histogram with statistics

### 3. Pickle Output (`--output-pickle`)

Serialized matplotlib figure for later loading and interactive exploration:

```python
import pickle
import matplotlib.pyplot as plt

with open('visualization.pkl', 'rb') as f:
    fig = pickle.load(f)

plt.show()
```

## Algorithm Details

### 3D Nearest-Neighbor Lane Matching

The tool automatically determines which OpenDRIVE lane boundary corresponds to each Lanelet2 boundary using spatial proximity:

1. Sample Lanelet2 boundary at regular intervals (default: 0.5m)
2. For each sample point, find the closest point among all OpenDRIVE lane boundaries (3D Euclidean distance)
3. Use majority voting to determine the matched OpenDRIVE lane
4. Calculate errors only between the matched pair

**Distance Calculation**: 3D Euclidean distance `sqrt((x₁-x₂)² + (y₁-y₂)² + (z₁-z₂)²)`

**Search Radius**: Maximum 10m (configurable in `config.py`)

### Error Calculation

- **Arc Length**: Calculated in 2D (XY plane) for consistency with OpenDRIVE S-coordinates
- **Distance**: Measured in 3D space for geometric accuracy
- **Sampling**: Regular intervals along S-coordinate (default: 0.5m)
- **Interpolation**: Linear interpolation between boundary points

## Interpreting Results

### Visualization

- **Blue segments**: Low error (good fit)
- **Red segments**: High error (poor fit)
- **Gray baseline**: Original Lanelet2 boundary

### Statistics

- **Mean error**: Average deviation across all sample points
- **95th percentile**: Error threshold below which 95% of points fall
- **Max error**: Worst-case deviation

### Typical Values

Based on `SplineConstants` in `config.py`:
- **Acceptable mean error**: < 2.0m (`max_avg_error`)
- **Acceptable max error**: < 8.0m (`max_point_error`)

### Error Thresholds

Configured in `VisualizationConstants`:
- **Warning threshold**: 0.5m
- **Critical threshold**: 1.0m

## Troubleshooting

### "No OpenDRIVE lane found within search radius"

**Cause**: Lanelet2 boundary is too far from all OpenDRIVE lane boundaries

**Solutions**:
1. Check coordinate system alignment (both files should use same origin/projection)
2. Verify MGRS projection code with `--mgrs-code`
3. Increase search radius in `config.py` (`nearest_neighbor_search_radius`)

### "No lane boundaries found in OpenDRIVE file"

**Cause**: OpenDRIVE parsing failed or file contains no lanes

**Solutions**:
1. Verify `.xodr` file is valid OpenDRIVE format
2. Check if file contains lane sections and lane definitions
3. Review pyxodr parsing logs for detailed error messages

### High Errors in Specific Regions

**Cause**: Spline fitting parameters may be suboptimal for high-curvature sections

**Solutions**:
1. Adjust spline fitting parameters in `config.py`:
   - Increase `control_points_ratio` for more control points
   - Adjust `curvature_multiplier` for better curve handling
2. Increase sampling density with `--sample-interval 0.1`

## Example Workflow

### 1. Initial Assessment

```bash
# Quick check with default settings
uv run visualize test/data/lanelet2_map.xodr test/data/lanelet2_map.osm
```

### 2. Detailed Analysis

```bash
# Save all outputs for documentation
uv run visualize \
    test/data/lanelet2_map.xodr \
    test/data/lanelet2_map.osm \
    --output-json error_analysis.json \
    --output-png boundary_comparison.png \
    --output-pickle visualization.pkl
```

### 3. Investigate Specific Lanelets

```bash
# Focus on high-error lanelets identified in step 2
uv run visualize \
    test/data/lanelet2_map.xodr \
    test/data/lanelet2_map.osm \
    --lanelet-id 120 121 122 \
    --sample-interval 0.1
```

### 4. Parameter Tuning

After identifying issues, adjust spline parameters in `src/autoware_lanelet2_to_opendrive/config.py`:

```python
@dataclass(frozen=True)
class SplineConstants:
    control_points_ratio: float = 0.5  # Increased from 0.4
    curvature_multiplier: float = 2.0  # Increased from 1.5
```

Then re-run conversion and visualization to verify improvements.

## Configuration

Constants are defined in `src/autoware_lanelet2_to_opendrive/config.py`:

```python
@dataclass(frozen=True)
class VisualizationConstants:
    sample_interval_default: float = 0.5  # Sampling interval (m)
    error_threshold_warning: float = 0.5  # Warning threshold (m)
    error_threshold_critical: float = 1.0  # Critical threshold (m)
    histogram_bins: int = 50  # Number of histogram bins
    figure_dpi: int = 300  # PNG resolution
    colormap_default: str = "coolwarm"  # Default colormap
    nearest_neighbor_search_radius: float = 10.0  # Max search radius (m)
```

## Performance

- **Processing time**: < 10 seconds for typical map (1000 lanelets, 100 lanes)
- **Bottleneck**: 3D nearest-neighbor search - O(M × N × P)
  - M = number of lanelet boundaries
  - N = number of sample points
  - P = number of OpenDRIVE lanes

## Limitations

### Current Limitations

1. **Coordinate System Alignment**: Assumes both files use the same coordinate origin and projection
2. **2D Lane Matching**: Matches lanes in XY plane; Z-coordinate not used for correspondence
3. **No Automatic Coordinate Transformation**: User must ensure coordinate system consistency

### Future Enhancements

- Automatic coordinate transformation support
- Interactive plot with selectable lanelets
- 3D visualization with elevation profiles
- Animation showing error evolution along road
- Comparison of multiple OpenDRIVE versions

## Related Documentation

- **Conversion Guide**: See main README for Lanelet2 to OpenDRIVE conversion workflow
- **Spline Configuration**: `CLAUDE.md` section on Constants Configuration
- **Coordinate Systems**: `CLAUDE.md` section on MGRS and coordinate projections

## References

- [OpenDRIVE Specification](https://www.asam.net/standards/detail/opendrive/)
- [Lanelet2 Documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [pyxodr Documentation](https://pypi.org/project/pyxodr/)

## Support

For issues or questions:
1. Check [GitHub Issues](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues)
2. Review `CLAUDE.md` for development guidelines
3. Enable debug logging: `export PYTHONLOGLEVEL=DEBUG`
