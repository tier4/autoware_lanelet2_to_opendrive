import numpy as np
from typing import Optional
from splines import CatmullRom


def point_to_line_segment_distance(
    point: np.ndarray,
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    direction: Optional[np.ndarray] = None,
) -> Optional[float]:
    """
    Calculate perpendicular distance from point to line segment in specified direction.

    Args:
        point: Point from which to measure distance
        seg_start: Start of line segment
        seg_end: End of line segment
        direction: Direction vector for perpendicular (if None, uses shortest distance)

    Returns:
        Distance if perpendicular intersects segment, None otherwise
    """
    # Use only 2D coordinates for all calculations
    seg_vec = seg_end[:2] - seg_start[:2]
    seg_length = np.linalg.norm(seg_vec)

    if seg_length < 1e-10:
        return np.linalg.norm(point[:2] - seg_start[:2])

    seg_unit = seg_vec / seg_length

    if direction is not None:
        direction_2d = direction[:2] / np.linalg.norm(direction[:2])

        # Find intersection of ray from point in given direction with infinite line
        # Line equation: seg_start + s * seg_unit
        # Ray equation: point + t * direction
        # Solve: seg_start + s * seg_unit = point + t * direction

        # Create 2x2 matrix: [seg_unit, -direction]
        # Solve for s and t
        A = np.column_stack([seg_unit, -direction_2d])
        b = point[:2] - seg_start[:2]

        try:
            # Solve linear system
            solution = np.linalg.solve(A, b)
            s, t = solution

            # Check if intersection is on the line segment (0 <= s <= seg_length)
            # and ray goes in positive direction (t >= 0)
            if 0 <= s <= seg_length and t >= 0:
                return t  # Distance along the ray
            else:
                return None
        except np.linalg.LinAlgError:
            # Matrix is singular (parallel lines)
            return None
    else:
        point_vec = point[:2] - seg_start[:2]
        projection = np.dot(point_vec, seg_unit)
        projection = np.clip(projection, 0, seg_length)

        closest = seg_start[:2] + projection * seg_unit
        return np.linalg.norm(point[:2] - closest)


class ArcLengthParameterizer:
    """Adapter to parameterize a CatmullRom spline by arc length."""

    def __init__(self, spline: CatmullRom, num_samples: int = 1000):
        self.spline = spline

        # Sample over the t range
        t_min = spline.grid[0]
        t_max = spline.grid[-1]
        self.t_values = np.linspace(t_min, t_max, num_samples)

        # Evaluate points at each t
        points = np.array([spline.evaluate(t).flatten() for t in self.t_values])

        # Compute cumulative arc length
        diffs = np.diff(points, axis=0)
        segment_lengths = np.linalg.norm(diffs, axis=1)
        self.arc_lengths = np.concatenate([[0], np.cumsum(segment_lengths)])

        self.total_length = self.arc_lengths[-1]

    def s_to_t(self, s: float) -> float:
        """Convert arc length s to parameter t."""
        # Clamp to valid range
        s = np.clip(s, 0, self.total_length)
        # Linearly interpolate to find t
        return np.interp(s, self.arc_lengths, self.t_values)

    def evaluate(self, s: float) -> np.ndarray:
        """Evaluate spline at arc length s."""
        t = self.s_to_t(s)
        return self.spline.evaluate(t)

    def evaluate_derivative(self, s: float, order: int = 1) -> np.ndarray:
        """Evaluate derivative of spline at arc length s."""
        t = self.s_to_t(s)

        # Get ds/dt using chain rule
        dt_ds = 1.0 / self._compute_speed_at_t(t)

        # Get dr/dt from the spline
        dr_dt = self.spline.evaluate(t, n=order)

        # Apply chain rule: dr/ds = (dr/dt) * (dt/ds)
        return dr_dt * dt_ds

    def _compute_speed_at_t(self, t: float) -> float:
        """Compute speed (ds/dt) at parameter t."""
        # Get velocity vector dr/dt
        velocity = self.spline.evaluate(t, n=1)
        # Speed is the magnitude of velocity
        return np.linalg.norm(velocity)


class ArcLengthParameterizedCatmullRomSpline:
    """
    Arc length parameterized Catmull-Rom spline with Frenet coordinate support.

    This class provides convenient access to spline evaluation using arc length
    parameters and supports Frenet coordinate calculations including tangent
    and normal vectors.
    """

    def __init__(self, points: np.ndarray, alpha: float = 0.5, num_samples: int = 1000):
        """
        Initialize arc length parameterized Catmull-Rom spline.

        Args:
            points: Array of control points (N x D) where N is number of points, D is dimensions
            alpha: Alpha parameter for Catmull-Rom spline (0=uniform, 0.5=centripetal, 1=chordal)
            num_samples: Number of samples used for arc length computation
        """
        if len(points) < 4:
            raise ValueError(
                "Points must have at least 4 points for Catmull-Rom spline. Use linear interpolation for fewer points."
            )

        # Create the base CatmullRom spline
        self._base_spline = CatmullRom(points, alpha=alpha)

        # Wrap it with arc length parameterization
        self._arc_length_spline = ArcLengthParameterizer(self._base_spline, num_samples)

    @property
    def total_length(self) -> float:
        """Get the total arc length of the spline."""
        return self._arc_length_spline.total_length

    def evaluate(self, s: float, frenet: bool = False) -> np.ndarray:
        """
        Evaluate the spline at arc length s.

        Args:
            s: Arc length parameter (0 to total_length)
            frenet: If True, return Frenet frame (position, tangent, normal)

        Returns:
            If frenet=False: position vector [x, y, z]
            If frenet=True: dictionary with 'position', 'tangent', 'normal'
        """
        if not frenet:
            return self._arc_length_spline.evaluate(s)

        # Calculate Frenet frame
        position = self._arc_length_spline.evaluate(s).flatten()

        # Calculate tangent using first derivative
        tangent = self._arc_length_spline.evaluate_derivative(s, order=1).flatten()

        # Normalize tangent
        tangent_magnitude = np.linalg.norm(tangent)
        if tangent_magnitude > 1e-10:
            tangent = tangent / tangent_magnitude
        else:
            raise ValueError(
                f"Degenerate tangent vector at arc length s={s:.6f}. "
                f"Tangent magnitude: {tangent_magnitude:.2e}. "
                "This indicates a singular point in the spline where the derivative is zero."
            )

        # Calculate normal (perpendicular to tangent in XY plane)
        normal = np.array([-tangent[1], tangent[0], 0.0])
        normal_magnitude = np.linalg.norm(normal)
        if normal_magnitude > 1e-10:
            normal = normal / normal_magnitude
        else:
            raise ValueError(
                f"Degenerate normal vector at arc length s={s:.6f}. "
                f"Normal magnitude: {normal_magnitude:.2e}. "
                "This indicates the tangent vector is parallel to the z-axis."
            )

        return {"position": position, "tangent": tangent, "normal": normal}

    def as_cubic_spline_parameters(self) -> list[dict]:
        """
        Export spline shape as cubic polynomial parameters using CatmullRom's native segments.

        Returns:
            List of dictionaries containing cubic polynomial parameters for each segment.
            Each dictionary contains:
            - 't_start': Start parameter t of the segment
            - 't_end': End parameter t of the segment
            - 's_start': Start arc length of the segment
            - 's_end': End arc length of the segment
            - 'a': Constant coefficient
            - 'b': Linear coefficient
            - 'c': Quadratic coefficient
            - 'd': Cubic coefficient
            - 'segment_length': Actual length of this segment
        """
        segments = []
        grid = self._base_spline.grid

        # CatmullRom has N-1 segments for N grid points
        # All segments from grid[0] to grid[-1] are valid for evaluation
        num_segments = len(grid) - 1

        for i in range(num_segments):
            t_start = grid[i]
            t_end = grid[i + 1]

            # Convert t parameters to arc lengths
            s_start = 0.0
            s_end = 0.0

            # Find corresponding arc lengths by searching through the parameterizer
            for j, t_val in enumerate(self._arc_length_spline.t_values):
                if abs(t_val - t_start) < 1e-6:
                    s_start = self._arc_length_spline.arc_lengths[j]
                if abs(t_val - t_end) < 1e-6:
                    s_end = self._arc_length_spline.arc_lengths[j]

            # If exact match not found, interpolate
            if s_start == 0.0:
                s_start = np.interp(
                    t_start,
                    self._arc_length_spline.t_values,
                    self._arc_length_spline.arc_lengths,
                )
            if s_end == 0.0:
                s_end = np.interp(
                    t_end,
                    self._arc_length_spline.t_values,
                    self._arc_length_spline.arc_lengths,
                )

            segment_length = s_end - s_start

            # Sample points within this t range for polynomial fitting
            num_samples = 20
            t_samples = np.linspace(t_start, t_end, num_samples)

            # Get positions in Frenet coordinate (relative to segment start)
            start_frame = self.evaluate(s_start, frenet=True)
            start_position = start_frame["position"]
            start_tangent = start_frame["tangent"]
            start_normal = start_frame["normal"]

            # Transform sampled points to local Frenet coordinates
            local_coords = []
            for t_val in t_samples:
                # Evaluate spline at t parameter directly
                global_pos = self._base_spline.evaluate(t_val).flatten()

                # Vector from segment start to current point
                delta = global_pos - start_position

                # Project onto Frenet frame
                local_s = np.dot(delta, start_tangent)  # Longitudinal coordinate
                local_t = np.dot(
                    delta, start_normal
                )  # Lateral coordinate (non-normalized t)

                local_coords.append([local_s, local_t])

            local_coords_array = np.array(local_coords)

            # Fit cubic polynomial: t = a + b*s + c*s^2 + d*s^3
            # Where s is local longitudinal coordinate, t is lateral coordinate (non-normalized)
            s_local = local_coords_array[:, 0]
            t_local = local_coords_array[:, 1]

            # Use actual segment length without normalization
            if len(s_local) >= 4:
                # Fit cubic polynomial using least squares
                # Create Vandermonde matrix [1, s, s^2, s^3] with actual s values
                A = np.vander(s_local, 4, increasing=True)

                try:
                    # Solve least squares problem
                    coeffs = np.linalg.lstsq(A, t_local, rcond=None)[0]
                    a, b, c, d = coeffs
                except np.linalg.LinAlgError:
                    # Fallback to linear interpolation if fitting fails
                    if len(t_local) >= 2:
                        s_range = s_local[-1] - s_local[0]
                        slope = (t_local[-1] - t_local[0]) / max(s_range, 1e-10)
                        a, b, c, d = t_local[0], slope, 0.0, 0.0
                    else:
                        a, b, c, d = (
                            t_local[0] if len(t_local) > 0 else 0.0,
                            0.0,
                            0.0,
                            0.0,
                        )
            else:
                # Not enough points for cubic fit, use constant
                a = np.mean(t_local) if len(t_local) > 0 else 0.0
                b, c, d = 0.0, 0.0, 0.0

            segments.append(
                {
                    "t_start": t_start,
                    "t_end": t_end,
                    "s_start": s_start,
                    "s_end": s_end,
                    "a": a,
                    "b": b,
                    "c": c,
                    "d": d,
                    "segment_length": segment_length,
                }
            )

        return segments
