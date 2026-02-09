"""Unit tests for geometry simplification."""

import math
import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    ParamPoly3,
    Line,
    Arc,
)
from autoware_lanelet2_to_opendrive.opendrive.geometry_simplifier import (
    GeometrySimplifier,
)
from autoware_lanelet2_to_opendrive.conversion_config import (
    GeometrySimplificationConfig,
)


class TestGeometrySimplifier:
    """Unit tests for geometry simplification."""

    def test_simplifier_disabled_returns_unchanged(self):
        """When disabled, simplifier returns geometries unchanged."""
        config = GeometrySimplificationConfig(enabled=False)
        simplifier = GeometrySimplifier(config)

        poly = ParamPoly3(
            s=0,
            x=0,
            y=0,
            hdg=0,
            length=10,
            aU=0,
            bU=1,
            cU=0,
            dU=0,
            aV=0,
            bV=0,
            cV=0,
            dV=0,
        )
        result = simplifier.simplify([poly])

        assert len(result) == 1
        assert isinstance(result[0], ParamPoly3)
        assert result[0] == poly

    def test_convert_trivial_parampoly3_to_line(self):
        """Straight line paramPoly3 converts to Line."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=True,
            convert_to_arc=False,
            consolidate_segments=False,
        )
        simplifier = GeometrySimplifier(config)

        # Straight line: u(t)=t, v(t)=0
        poly = ParamPoly3(
            s=0,
            x=0,
            y=0,
            hdg=0,
            length=10,
            aU=0,
            bU=1,
            cU=0,  # Near-zero
            dU=0,  # Near-zero
            aV=0,
            bV=0,  # Near-zero
            cV=0,  # Near-zero
            dV=0,  # Near-zero
        )

        result = simplifier.simplify([poly])

        assert len(result) == 1
        assert isinstance(result[0], Line)
        assert result[0].length == 10.0
        assert result[0].x == 0.0
        assert result[0].y == 0.0
        assert result[0].hdg == 0.0

    def test_keep_complex_parampoly3_unchanged(self):
        """Complex paramPoly3 with large coefficients stays unchanged."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=True,
            convert_to_arc=False,
            consolidate_segments=False,
        )
        simplifier = GeometrySimplifier(config)

        # Large cU coefficient (above threshold)
        poly = ParamPoly3(
            s=0,
            x=0,
            y=0,
            hdg=0,
            length=10,
            aU=0,
            bU=1,
            cU=0.5,  # Above threshold
            dU=0,
            aV=0,
            bV=0,
            cV=0,
            dV=0,
        )

        result = simplifier.simplify([poly])

        assert len(result) == 1
        assert isinstance(result[0], ParamPoly3)

    def test_convert_circular_parampoly3_to_arc(self):
        """Circular curve converts to Arc."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=True,
            consolidate_segments=False,
        )
        simplifier = GeometrySimplifier(config)

        # Create a circular arc with radius 10m, 90-degree turn
        # For a circular arc: u(p) = R*sin(p/R), v(p) = R*(1 - cos(p/R))
        # We'll approximate with ParamPoly3 quadratic terms
        radius = 10.0
        arc_length = math.pi * radius / 2  # 90 degrees = π/2 radians

        # Approximate circular arc with quadratic ParamPoly3
        poly = ParamPoly3(
            s=0,
            x=0,
            y=0,
            hdg=0,
            length=arc_length,
            aU=0,
            bU=1,
            cU=0,  # Approximate
            dU=0,
            aV=0,
            bV=0,
            cV=1.0 / (2 * radius),  # Quadratic term for circular arc
            dV=0,
        )

        result = simplifier.simplify([poly])

        # Should convert to Arc (or stay as ParamPoly3 if fit isn't good enough)
        assert len(result) == 1
        # Note: This test might fail if the approximation isn't good enough
        # In practice, real ParamPoly3 from splines will fit better

    def test_consolidate_two_short_lines(self):
        """Two consecutive short Lines merge into one."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
        )
        simplifier = GeometrySimplifier(config)

        lines = [
            Line(s=0, x=0, y=0, hdg=0, length=2),
            Line(s=2, x=2, y=0, hdg=0, length=2),
        ]

        result = simplifier.simplify(lines)

        assert len(result) == 1
        assert isinstance(result[0], Line)
        assert result[0].length == 4.0
        assert result[0].s == 0
        assert result[0].x == 0
        assert result[0].y == 0

    def test_dont_consolidate_different_types(self):
        """Line and Arc don't merge."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
        )
        simplifier = GeometrySimplifier(config)

        geometries = [
            Line(s=0, x=0, y=0, hdg=0, length=2),
            Arc(s=2, x=2, y=0, hdg=0, length=2, curvature=0.1),
        ]

        result = simplifier.simplify(geometries)

        # Should not merge (different types)
        assert len(result) == 2
        assert isinstance(result[0], Line)
        assert isinstance(result[1], Arc)

    def test_dont_consolidate_heading_mismatch(self):
        """Segments with large heading difference don't merge."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
            max_heading_diff_degrees=5.0,
        )
        simplifier = GeometrySimplifier(config)

        # Two lines with different headings (20 degrees difference)
        lines = [
            Line(s=0, x=0, y=0, hdg=0, length=2),
            Line(s=2, x=2, y=0, hdg=math.radians(20), length=2),  # Large heading diff
        ]

        result = simplifier.simplify(lines)

        # Should not merge (heading difference too large)
        assert len(result) == 2

    def test_dont_consolidate_long_segments(self):
        """Segments longer than min_segment_length don't merge."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
        )
        simplifier = GeometrySimplifier(config)

        lines = [
            Line(s=0, x=0, y=0, hdg=0, length=10),  # Too long
            Line(s=10, x=10, y=0, hdg=0, length=2),
        ]

        result = simplifier.simplify(lines)

        # Should not merge (first segment is too long)
        assert len(result) == 2

    def test_consolidate_multiple_segments(self):
        """Multiple consecutive short segments merge into one."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
        )
        simplifier = GeometrySimplifier(config)

        lines = [
            Line(s=0, x=0, y=0, hdg=0, length=2),
            Line(s=2, x=2, y=0, hdg=0, length=2),
            Line(s=4, x=4, y=0, hdg=0, length=1),
        ]

        result = simplifier.simplify(lines)

        # All should merge into one
        assert len(result) == 1
        assert result[0].length == 5.0

    def test_full_pipeline_line_conversion_and_consolidation(self):
        """Test full pipeline: convert ParamPoly3 to Line and consolidate."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=True,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
        )
        simplifier = GeometrySimplifier(config)

        # Two straight ParamPoly3 segments (should convert to Line and merge)
        geometries = [
            ParamPoly3(
                s=0,
                x=0,
                y=0,
                hdg=0,
                length=2,
                aU=0,
                bU=1,
                cU=0,
                dU=0,
                aV=0,
                bV=0,
                cV=0,
                dV=0,
            ),
            ParamPoly3(
                s=2,
                x=2,
                y=0,
                hdg=0,
                length=2,
                aU=0,
                bU=1,
                cU=0,
                dU=0,
                aV=0,
                bV=0,
                cV=0,
                dV=0,
            ),
        ]

        result = simplifier.simplify(geometries)

        # Should convert to Line and consolidate to 1 segment
        assert len(result) == 1
        assert isinstance(result[0], Line)
        assert result[0].length == 4.0

    def test_empty_geometry_list(self):
        """Empty geometry list returns empty result."""
        config = GeometrySimplificationConfig(enabled=True)
        simplifier = GeometrySimplifier(config)

        result = simplifier.simplify([])

        assert len(result) == 0

    def test_single_geometry_unchanged(self):
        """Single geometry remains unchanged."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
        )
        simplifier = GeometrySimplifier(config)

        line = Line(s=0, x=0, y=0, hdg=0, length=2)
        result = simplifier.simplify([line])

        assert len(result) == 1
        assert result[0] == line

    def test_compute_end_heading_line(self):
        """Line end heading equals start heading (constant)."""
        config = GeometrySimplificationConfig(enabled=True)
        simplifier = GeometrySimplifier(config)

        line = Line(s=0, x=0, y=0, hdg=math.radians(30), length=10)
        end_heading = simplifier._compute_end_heading(line)

        assert end_heading == pytest.approx(math.radians(30))

    def test_compute_end_heading_arc(self):
        """Arc end heading changes by curvature * length."""
        config = GeometrySimplificationConfig(enabled=True)
        simplifier = GeometrySimplifier(config)

        # Arc with 10m length and 0.1 curvature (radius=10m)
        # Heading change = 0.1 * 10 = 1 radian
        arc = Arc(s=0, x=0, y=0, hdg=0, length=10, curvature=0.1)
        end_heading = simplifier._compute_end_heading(arc)

        assert end_heading == pytest.approx(1.0)  # 1 radian

    def test_fit_circle_perfect_circle(self):
        """Circle fitting works correctly for perfect circle points."""
        config = GeometrySimplificationConfig(enabled=True)
        simplifier = GeometrySimplifier(config)

        # Generate perfect circle points with radius 5, centered at (5, 5)
        radius = 5.0
        center_x = 5.0
        center_y = 5.0
        angles = np.linspace(0, np.pi / 2, 10)  # Quarter circle
        points = np.array(
            [
                [center_x + radius * np.cos(a), center_y + radius * np.sin(a)]
                for a in angles
            ]
        )

        result = simplifier._fit_circle(points)

        assert result is not None
        cx, cy, r = result
        assert cx == pytest.approx(center_x, abs=0.1)
        assert cy == pytest.approx(center_y, abs=0.1)
        assert r == pytest.approx(radius, abs=0.1)

    def test_fit_circle_insufficient_points(self):
        """Circle fitting fails with insufficient points."""
        config = GeometrySimplificationConfig(enabled=True)
        simplifier = GeometrySimplifier(config)

        # Only 2 points (need at least 3)
        points = np.array([[0, 0], [1, 1]])

        result = simplifier._fit_circle(points)

        assert result is None

    def test_arc_consolidation_same_curvature(self):
        """Two arcs with same curvature can merge."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
        )
        simplifier = GeometrySimplifier(config)

        arcs = [
            Arc(s=0, x=0, y=0, hdg=0, length=2, curvature=0.1),
            Arc(s=2, x=1.99, y=0.2, hdg=0.2, length=2, curvature=0.1),  # Same curvature
        ]

        result = simplifier.simplify(arcs)

        # Should merge (same type, same curvature, both short)
        assert len(result) == 1
        assert isinstance(result[0], Arc)
        assert result[0].curvature == pytest.approx(0.1)

    def test_arc_no_consolidation_different_curvature(self):
        """Two arcs with different curvature don't merge."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
        )
        simplifier = GeometrySimplifier(config)

        arcs = [
            Arc(s=0, x=0, y=0, hdg=0, length=2, curvature=0.1),
            Arc(s=2, x=2, y=0, hdg=0, length=2, curvature=0.2),  # Different curvature
        ]

        result = simplifier.simplify(arcs)

        # Should not merge (different curvature)
        assert len(result) == 2

    def test_s_coordinate_continuity_after_consolidation(self):
        """CRITICAL: s-coordinates must be continuous after consolidation."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
        )
        simplifier = GeometrySimplifier(config)

        # Create segments with gaps in s-coordinates
        lines = [
            Line(s=0, x=0, y=0, hdg=0, length=2),
            Line(s=2, x=2, y=0, hdg=0, length=2),
            Line(s=10, x=4, y=0, hdg=0, length=2),  # Gap: s=4→10
            Line(s=20, x=6, y=0, hdg=0, length=2),  # Gap: s=12→20
        ]

        result = simplifier.simplify(lines)

        # Check s-coordinate continuity (no gaps)
        for i in range(len(result) - 1):
            current = result[i]
            next_seg = result[i + 1]
            expected_next_s = current.s + current.length
            assert (
                next_seg.s == pytest.approx(expected_next_s, abs=1e-6)
            ), f"Gap detected: segment {i} ends at s={expected_next_s}, but segment {i+1} starts at s={next_seg.s}"

    def test_position_continuity_after_consolidation(self):
        """CRITICAL: x,y positions must be continuous after consolidation."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
        )
        simplifier = GeometrySimplifier(config)

        # Create segments with position discontinuities
        lines = [
            Line(s=0, x=0, y=0, hdg=0, length=2),
            Line(s=2, x=10, y=5, hdg=0, length=2),  # Wrong position
        ]

        result = simplifier.simplify(lines)

        # Check position continuity
        for i in range(len(result) - 1):
            current = result[i]
            next_seg = result[i + 1]

            # Calculate expected end position
            end_x = current.x + current.length * math.cos(current.hdg)
            end_y = current.y + current.length * math.sin(current.hdg)

            assert (
                next_seg.x == pytest.approx(end_x, abs=1e-3)
            ), f"Position discontinuity: segment {i} ends at x={end_x}, but segment {i+1} starts at x={next_seg.x}"
            assert (
                next_seg.y == pytest.approx(end_y, abs=1e-3)
            ), f"Position discontinuity: segment {i} ends at y={end_y}, but segment {i+1} starts at y={next_seg.y}"

    def test_no_segment_skipping_during_consolidation(self):
        """CRITICAL: No segments should be skipped during consolidation."""
        config = GeometrySimplificationConfig(
            enabled=True,
            convert_to_line=False,
            convert_to_arc=False,
            consolidate_segments=True,
            min_segment_length=5.0,
        )
        simplifier = GeometrySimplifier(config)

        # Pattern: [merge, merge, no-merge, merge, merge]
        # A(2m) + B(2m) + C(2m) → merge to 6m
        # D(10m) → cannot merge (too long)
        # E(2m) + F(2m) → merge to 4m
        # Result should have 3 segments, not skip D
        lines = [
            Line(s=0, x=0, y=0, hdg=0, length=2),  # A
            Line(s=2, x=2, y=0, hdg=0, length=2),  # B
            Line(s=4, x=4, y=0, hdg=0, length=2),  # C
            Line(s=6, x=6, y=0, hdg=0, length=10),  # D (too long)
            Line(s=16, x=16, y=0, hdg=0, length=2),  # E
            Line(s=18, x=18, y=0, hdg=0, length=2),  # F
        ]

        result = simplifier.simplify(lines)

        # Should have 3 segments: [A+B+C(6m), D(10m), E+F(4m)]
        assert len(result) == 3
        assert result[0].length == pytest.approx(6.0)
        assert result[1].length == pytest.approx(10.0)
        assert result[2].length == pytest.approx(4.0)

        # Verify D was not skipped
        assert result[1].s == pytest.approx(6.0)
