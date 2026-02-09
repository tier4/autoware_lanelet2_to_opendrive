"""Geometry simplification for OpenDRIVE output.

This module simplifies ParamPoly3 geometries by converting trivial segments to
simpler geometry types (Line/Arc) and consolidating short consecutive segments.
"""

import math
import numpy as np
from typing import List, Optional, Tuple
import logging

from .geometry import GeometryBase, Line, Arc, ParamPoly3
from ..conversion_config import GeometrySimplificationConfig

logger = logging.getLogger(__name__)


class GeometrySimplifier:
    """Simplifies ParamPoly3 geometries by converting trivial segments to Line/Arc.

    This simplifier reduces OpenDRIVE file complexity and improves simulator
    compatibility by:
    1. Converting straight ParamPoly3 segments to Line geometry
    2. Converting circular ParamPoly3 segments to Arc geometry
    3. Consolidating short consecutive segments of the same type

    All simplifications are controlled by the GeometrySimplificationConfig.
    """

    def __init__(self, config: GeometrySimplificationConfig):
        """Initialize the geometry simplifier.

        Args:
            config: Configuration controlling simplification behavior
        """
        self.config = config

    def simplify(self, geometries: List[GeometryBase]) -> List[GeometryBase]:
        """Main entry point for geometry simplification.

        Args:
            geometries: List of geometry segments (typically ParamPoly3)

        Returns:
            Simplified list of geometry segments (mixed Line/Arc/ParamPoly3)
        """
        if not self.config.enabled:
            return geometries

        if not geometries:
            return geometries

        result = geometries.copy()

        # Step 1: Convert trivial paramPoly3 to Line/Arc
        if self.config.convert_to_line or self.config.convert_to_arc:
            result = self._convert_trivial_parampoly3(result)
            logger.debug(
                f"Geometry conversion: {len(geometries)} -> {len(result)} segments"
            )

        # Step 2: Consolidate short consecutive segments
        if self.config.consolidate_segments:
            pre_consolidation = len(result)
            result = self._consolidate_segments(result)
            logger.debug(
                f"Segment consolidation: {pre_consolidation} -> {len(result)} segments"
            )

        return result

    def _convert_trivial_parampoly3(
        self, geometries: List[GeometryBase]
    ) -> List[GeometryBase]:
        """Convert paramPoly3 with near-zero coefficients to Line or Arc.

        Args:
            geometries: List of geometry segments

        Returns:
            List with trivial ParamPoly3 converted to Line or Arc
        """
        result = []
        line_count = 0
        arc_count = 0

        for geom in geometries:
            if not isinstance(geom, ParamPoly3):
                result.append(geom)
                continue

            # Try Line conversion first (simpler geometry)
            if self.config.convert_to_line:
                line_geom = self._try_convert_to_line(geom)
                if line_geom is not None:
                    result.append(line_geom)
                    line_count += 1
                    continue

            # Try Arc conversion
            if self.config.convert_to_arc:
                arc_geom = self._try_convert_to_arc(geom)
                if arc_geom is not None:
                    result.append(arc_geom)
                    arc_count += 1
                    continue

            # Keep as ParamPoly3 if no conversion applied
            result.append(geom)

        if line_count > 0 or arc_count > 0:
            logger.debug(
                f"Converted {line_count} ParamPoly3 to Line, {arc_count} to Arc"
            )

        return result

    def _try_convert_to_line(self, pp3: ParamPoly3) -> Optional[Line]:
        """Try converting paramPoly3 to Line if coefficients are near-zero.

        A ParamPoly3 represents a line if:
        - |cU| < threshold (no quadratic term in u)
        - |dU| < threshold (no cubic term in u)
        - |bV| < 0.01 (nearly zero lateral velocity)
        - |cV| < threshold (no quadratic term in v)
        - |dV| < threshold (no cubic term in v)

        Args:
            pp3: ParamPoly3 segment to check

        Returns:
            Line geometry if conversion is valid, None otherwise
        """
        # Check if higher-order coefficients are near-zero
        if abs(pp3.cU) > self.config.line_cu_threshold:
            return None
        if abs(pp3.dU) > self.config.line_du_threshold:
            return None
        if abs(pp3.cV) > self.config.line_cv_threshold:
            return None
        if abs(pp3.dV) > self.config.line_dv_threshold:
            return None

        # Check if lateral velocity is near-zero (straight line)
        if abs(pp3.bV) > 0.01:
            return None

        # Valid line: create Line geometry
        return Line(
            s=pp3.s,
            x=pp3.x,
            y=pp3.y,
            hdg=pp3.hdg,
            length=pp3.length,
        )

    def _try_convert_to_arc(self, pp3: ParamPoly3) -> Optional[Arc]:
        """Try fitting paramPoly3 as Arc using algebraic circle fit.

        Samples points along the ParamPoly3 curve, fits a circle using
        algebraic least-squares, and checks if the fit is good enough.

        Args:
            pp3: ParamPoly3 segment to check

        Returns:
            Arc geometry if fit is good, None otherwise
        """
        # Sample points along the curve
        num_samples = 10
        t_values = np.linspace(0, 1, num_samples)
        points = []

        for t in t_values:
            # Evaluate ParamPoly3 in local coordinates
            p = pp3.length * t if pp3.pRange == "arcLength" else t
            u = pp3.aU + pp3.bU * p + pp3.cU * p**2 + pp3.dU * p**3
            v = pp3.aV + pp3.bV * p + pp3.cV * p**2 + pp3.dV * p**3

            # Transform to global coordinates
            cos_hdg = math.cos(pp3.hdg)
            sin_hdg = math.sin(pp3.hdg)
            x = pp3.x + u * cos_hdg - v * sin_hdg
            y = pp3.y + u * sin_hdg + v * cos_hdg
            points.append((x, y))

        points = np.array(points)

        # Algebraic circle fit
        circle_params = self._fit_circle(points)
        if circle_params is None:
            return None

        cx, cy, radius = circle_params

        # Validate curvature error
        if radius < 1e-6:  # Avoid division by zero
            return None

        fitted_curvature = 1.0 / radius
        curvature_error = self._compute_curvature_error(pp3, fitted_curvature)

        if curvature_error > self.config.arc_curvature_error_threshold:
            return None

        # Validate position error
        position_error = self._compute_position_error(points, cx, cy, radius)
        if position_error > self.config.arc_position_error_threshold:
            return None

        # Determine curvature sign (left turn: positive, right turn: negative)
        # Use cross product of start tangent and vector to center
        start_tangent = np.array([math.cos(pp3.hdg), math.sin(pp3.hdg)])
        to_center = np.array([cx - pp3.x, cy - pp3.y])
        cross = start_tangent[0] * to_center[1] - start_tangent[1] * to_center[0]
        curvature = fitted_curvature if cross > 0 else -fitted_curvature

        # Valid arc: create Arc geometry
        return Arc(
            s=pp3.s,
            x=pp3.x,
            y=pp3.y,
            hdg=pp3.hdg,
            length=pp3.length,
            curvature=curvature,
        )

    def _fit_circle(self, points: np.ndarray) -> Optional[Tuple[float, float, float]]:
        """Fit a circle to points using algebraic least-squares.

        Args:
            points: Nx2 array of (x, y) coordinates

        Returns:
            (cx, cy, radius) if successful, None otherwise
        """
        if len(points) < 3:
            return None

        # Algebraic circle fit: minimize ||A @ params - b||^2
        # where (x - cx)^2 + (y - cy)^2 = r^2
        # Rearrange: x^2 + y^2 = 2*cx*x + 2*cy*y + (r^2 - cx^2 - cy^2)
        A = np.column_stack([points[:, 0], points[:, 1], np.ones(len(points))])
        b = points[:, 0] ** 2 + points[:, 1] ** 2

        try:
            # Solve using least-squares
            params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            cx = params[0] / 2
            cy = params[1] / 2
            radius_sq = params[2] + cx**2 + cy**2

            if radius_sq < 0:
                return None

            radius = math.sqrt(radius_sq)
            return cx, cy, radius
        except np.linalg.LinAlgError:
            return None

    def _compute_curvature_error(
        self, pp3: ParamPoly3, fitted_curvature: float
    ) -> float:
        """Compute maximum curvature error between ParamPoly3 and fitted arc.

        Args:
            pp3: ParamPoly3 segment
            fitted_curvature: Curvature of fitted arc (absolute value)

        Returns:
            Maximum absolute curvature error
        """
        # Sample curvature along ParamPoly3
        t_values = np.linspace(0, 1, 10)
        max_error = 0.0

        for t in t_values:
            p = pp3.length * t if pp3.pRange == "arcLength" else t

            # Compute derivatives
            du_dp = pp3.bU + 2 * pp3.cU * p + 3 * pp3.dU * p**2
            dv_dp = pp3.bV + 2 * pp3.cV * p + 3 * pp3.dV * p**2
            d2u_dp2 = 2 * pp3.cU + 6 * pp3.dU * p
            d2v_dp2 = 2 * pp3.cV + 6 * pp3.dV * p

            # Compute curvature using formula: κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
            denominator = (du_dp**2 + dv_dp**2) ** 1.5
            if denominator < 1e-10:
                continue

            curvature = abs((du_dp * d2v_dp2 - dv_dp * d2u_dp2) / denominator)
            error = abs(curvature - fitted_curvature)
            max_error = max(max_error, error)

        return max_error

    def _compute_position_error(
        self, points: np.ndarray, cx: float, cy: float, radius: float
    ) -> float:
        """Compute maximum position error from fitted circle.

        Args:
            points: Nx2 array of sampled points
            cx, cy: Circle center coordinates
            radius: Circle radius

        Returns:
            Maximum distance error from circle (meters)
        """
        distances = np.sqrt((points[:, 0] - cx) ** 2 + (points[:, 1] - cy) ** 2)
        errors = np.abs(distances - radius)
        return float(np.max(errors))

    def _consolidate_segments(
        self, geometries: List[GeometryBase]
    ) -> List[GeometryBase]:
        """Merge consecutive short segments of the same type.

        Uses a greedy algorithm: iterate through segments and merge consecutive
        segments if they are the same type, both short, and have small heading
        difference.

        Args:
            geometries: List of geometry segments

        Returns:
            List with consecutive short segments merged
        """
        if len(geometries) <= 1:
            return geometries

        result = []
        i = 0

        while i < len(geometries):
            current = geometries[i]

            # Try to merge with next segment(s)
            merged = False
            for j in range(i + 1, len(geometries)):
                next_geom = geometries[j]

                if self._can_merge(current, next_geom):
                    # Merge: extend current segment length
                    current = self._merge_geometries(current, next_geom)
                    merged = True
                else:
                    # Cannot merge: commit current and move to next
                    break

            result.append(current)

            # Skip merged segments
            if merged:
                i = j + 1
            else:
                i += 1

        return result

    def _can_merge(self, geom1: GeometryBase, geom2: GeometryBase) -> bool:
        """Check if two consecutive segments can be merged.

        Two segments can merge if:
        - They are the same type (both Line, both Arc with same curvature, etc.)
        - Both are shorter than min_segment_length
        - s-coordinates are continuous (geom2.s ≈ geom1.s + geom1.length)
        - Heading difference is small

        Args:
            geom1: First geometry segment
            geom2: Second geometry segment

        Returns:
            True if segments can be merged
        """
        # Must be same type
        if type(geom1) is not type(geom2):
            return False

        # Both must be short
        if (
            geom1.length >= self.config.min_segment_length
            or geom2.length >= self.config.min_segment_length
        ):
            return False

        # Check s-coordinate continuity
        s_gap = abs(geom2.s - (geom1.s + geom1.length))
        if s_gap > 1e-6:
            return False

        # Check heading continuity
        end_heading1 = self._compute_end_heading(geom1)
        heading_diff = abs(end_heading1 - geom2.hdg)
        # Normalize to [-π, π]
        heading_diff = (heading_diff + math.pi) % (2 * math.pi) - math.pi
        heading_diff_deg = abs(math.degrees(heading_diff))

        if heading_diff_deg > self.config.max_heading_diff_degrees:
            return False

        # For arcs, curvature must match
        if isinstance(geom1, Arc) and isinstance(geom2, Arc):
            if abs(geom1.curvature - geom2.curvature) > 0.01:
                return False

        return True

    def _merge_geometries(
        self, geom1: GeometryBase, geom2: GeometryBase
    ) -> GeometryBase:
        """Merge two consecutive segments into one.

        Args:
            geom1: First geometry segment
            geom2: Second geometry segment

        Returns:
            Merged geometry segment
        """
        # Create new geometry with combined length
        # Keep starting position and heading from geom1
        if isinstance(geom1, Line):
            return Line(
                s=geom1.s,
                x=geom1.x,
                y=geom1.y,
                hdg=geom1.hdg,
                length=geom1.length + geom2.length,
            )
        elif isinstance(geom1, Arc):
            return Arc(
                s=geom1.s,
                x=geom1.x,
                y=geom1.y,
                hdg=geom1.hdg,
                length=geom1.length + geom2.length,
                curvature=geom1.curvature,
            )
        elif isinstance(geom1, ParamPoly3):
            # Cannot easily merge ParamPoly3 - return first segment
            # This should rarely happen in practice
            logger.warning("Cannot merge ParamPoly3 segments - keeping first")
            return geom1
        else:
            return geom1

    def _compute_end_heading(self, geom: GeometryBase) -> float:
        """Compute the heading at the end of a geometry segment.

        Args:
            geom: Geometry segment

        Returns:
            Heading at the end of the segment (radians)
        """
        if isinstance(geom, Line):
            # Line has constant heading
            return geom.hdg
        elif isinstance(geom, Arc):
            # Arc heading changes by: Δhdg = curvature * length
            return geom.hdg + geom.curvature * geom.length
        elif isinstance(geom, ParamPoly3):
            # Compute tangent at end of ParamPoly3
            p = geom.length if geom.pRange == "arcLength" else 1.0
            du_dp = geom.bU + 2 * geom.cU * p + 3 * geom.dU * p**2
            dv_dp = geom.bV + 2 * geom.cV * p + 3 * geom.dV * p**2
            local_heading = math.atan2(dv_dp, du_dp)
            return geom.hdg + local_heading
        else:
            return geom.hdg
