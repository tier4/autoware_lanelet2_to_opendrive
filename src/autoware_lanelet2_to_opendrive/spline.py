"""B-Spline interpolation with constrained fitting."""

import numpy as np
from typing import Optional
from scipy.interpolate import BSpline


class Splines:
    """
    B-Spline interpolation class using constrained fitting algorithm.

    This class creates a B-spline curve that approximates a 3D point sequence with:
    - Hard constraints on start/end positions and tangent vectors
    - Soft constraints for intermediate points
    """

    def __init__(
        self,
        points: np.ndarray,
        start_vel: Optional[np.ndarray] = None,
        end_vel: Optional[np.ndarray] = None,
        num_control_points: int = 10,
        k: int = 3,
    ):
        """
        Initialize a B-spline with constrained fitting.

        Args:
            points: (N, 3) Array of 3D points to fit
            start_vel: (3,) Tangent vector at the start. If None, estimated from points.
            end_vel: (3,) Tangent vector at the end. If None, estimated from points.
            num_control_points: Number of control points (higher = closer fit, lower = smoother)
            k: Degree of the spline (usually 3 for cubic)

        Raises:
            ValueError: If too few control points or invalid input
        """
        self.points = np.asarray(points)
        if self.points.ndim != 2 or self.points.shape[1] != 3:
            raise ValueError("Points must be an (N, 3) array")

        self.n_points = len(self.points)
        if self.n_points < 2:
            raise ValueError("At least 2 points are required")

        self.k = k
        self.num_control_points = num_control_points

        # Estimate tangent vectors if not provided
        if start_vel is None:
            if self.n_points >= 3:
                # Use second-order estimation for better accuracy
                start_vel = self.points[1] - self.points[0]
                start_norm = np.linalg.norm(start_vel)
                if start_norm > 1e-10:
                    start_vel = start_vel / start_norm
                else:
                    # Fall back to longer distance if points are duplicate
                    start_vel = self.points[2] - self.points[0]
                    start_norm = np.linalg.norm(start_vel)
                    if start_norm > 1e-10:
                        start_vel = start_vel / start_norm
                    else:
                        start_vel = np.array([1.0, 0.0, 0.0])
            elif self.n_points >= 2:
                start_vel = self.points[1] - self.points[0]
                start_norm = np.linalg.norm(start_vel)
                if start_norm > 1e-10:
                    start_vel = start_vel / start_norm
                else:
                    start_vel = np.array([1.0, 0.0, 0.0])
            else:
                start_vel = np.array([1.0, 0.0, 0.0])

        if end_vel is None:
            if self.n_points >= 3:
                # Use second-order estimation for better accuracy
                end_vel = self.points[-1] - self.points[-2]
                end_norm = np.linalg.norm(end_vel)
                if end_norm > 1e-10:
                    end_vel = end_vel / end_norm
                else:
                    # Fall back to longer distance if points are duplicate
                    end_vel = self.points[-1] - self.points[-3]
                    end_norm = np.linalg.norm(end_vel)
                    if end_norm > 1e-10:
                        end_vel = end_vel / end_norm
                    else:
                        end_vel = np.array([1.0, 0.0, 0.0])
            elif self.n_points >= 2:
                end_vel = self.points[-1] - self.points[-2]
                end_norm = np.linalg.norm(end_vel)
                if end_norm > 1e-10:
                    end_vel = end_vel / end_norm
                else:
                    end_vel = np.array([1.0, 0.0, 0.0])
            else:
                end_vel = np.array([1.0, 0.0, 0.0])

        self.start_vel = np.asarray(start_vel)
        self.end_vel = np.asarray(end_vel)

        # Perform constrained spline fitting
        self._fit_constrained_spline()

    def _fit_constrained_spline(self):
        """
        Internal method to perform the constrained B-spline fitting.
        """
        # 1. Parameterization (Chord length parameterization)
        dists = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.t_data = np.concatenate(([0], np.cumsum(dists)))
        self.t_max = self.t_data[-1]

        # Handle zero length case
        if self.t_max == 0:
            self.t_max = 1.0
            self.t_data = np.linspace(0, 1, self.n_points)
        else:
            self.t_data /= self.t_max  # Normalize to 0.0 ~ 1.0

        # 2. Create knot vector (Clamped knots)
        n_internal = self.num_control_points - (self.k + 1)
        if n_internal < 0:
            raise ValueError(f"Too few control points. Must be at least {self.k + 1}.")

        self.knots = np.concatenate(
            [
                np.zeros(self.k + 1),
                np.linspace(0, 1, n_internal + 2)[1:-1] if n_internal > 0 else [],
                np.ones(self.k + 1),
            ]
        )

        # 3. Build Design Matrix
        # Data fitting term (Soft constraint)
        A_fit = self._get_basis_matrix(self.t_data, deriv=0)
        b_fit = self.points

        # Boundary condition terms (Hard constraints)
        # Positions at t=0, t=1
        A_pos_start = self._get_basis_matrix([0.0], deriv=0)
        A_pos_end = self._get_basis_matrix([1.0], deriv=0)

        # Derivatives (Tangent vectors) at t=0, t=1
        A_vel_start = self._get_basis_matrix([0.0], deriv=1)
        A_vel_end = self._get_basis_matrix([1.0], deriv=1)

        # 4. Setup Least Squares with weights
        w_hard = 100.0  # Weight for hard constraints (reduced from 1e4)
        w_soft = 1.0  # Weight for data fitting

        # Combine matrices
        A_combined = np.vstack(
            [
                A_fit * w_soft,
                A_pos_start * w_hard,
                A_pos_end * w_hard,
                A_vel_start * w_hard,
                A_vel_end * w_hard,
            ]
        )

        # Combine targets (scale velocities by t_max for normalized parameter space)
        b_combined = np.vstack(
            [
                b_fit * w_soft,
                self.points[[0]] * w_hard,  # Start position
                self.points[[-1]] * w_hard,  # End position
                (self.start_vel * self.t_max * w_hard).reshape(
                    1, -1
                ),  # Start velocity (scaled)
                (self.end_vel * self.t_max * w_hard).reshape(
                    1, -1
                ),  # End velocity (scaled)
            ]
        )

        # 5. Solve for control points (Least Squares)
        coeffs_x, _, _, _ = np.linalg.lstsq(A_combined, b_combined[:, 0], rcond=None)
        coeffs_y, _, _, _ = np.linalg.lstsq(A_combined, b_combined[:, 1], rcond=None)
        coeffs_z, _, _, _ = np.linalg.lstsq(A_combined, b_combined[:, 2], rcond=None)

        self.coeffs = np.column_stack([coeffs_x, coeffs_y, coeffs_z])

        # Create the B-Spline object
        self.spline = BSpline(self.knots, self.coeffs, self.k)

    def _get_basis_matrix(self, t_vals: np.ndarray, deriv: int = 0) -> np.ndarray:
        """
        Calculate basis function values at given parameter values.

        Args:
            t_vals: Parameter values to evaluate at
            deriv: Derivative order (0 for position, 1 for velocity, etc.)

        Returns:
            Matrix of basis function values
        """
        mat = []
        dummy_coeffs = np.eye(self.num_control_points)
        for i in range(self.num_control_points):
            bs = BSpline(self.knots, dummy_coeffs[i], self.k)
            val = bs(t_vals, nu=deriv)  # nu = derivative order
            mat.append(val)
        return np.array(mat).T

    def evaluate(self, t: float, derivative: int = 0) -> np.ndarray:
        """
        Evaluate the spline at a given parameter value.

        Args:
            t: Parameter value (0.0 to 1.0)
            derivative: Derivative order (0=position, 1=velocity, 2=acceleration)

        Returns:
            3D point or derivative vector at parameter t
        """
        return self.spline(t, nu=derivative)

    def _compute_arc_length_table(self, num_samples: int = 1000):
        """Compute arc length lookup table for accurate parameterization."""
        if hasattr(self, "_arc_length_table"):
            return

        t_vals = np.linspace(0.0, 1.0, num_samples)
        positions = np.array([self.evaluate(t) for t in t_vals])

        # Compute arc lengths
        distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        arc_lengths = np.concatenate(([0], np.cumsum(distances)))

        self._arc_length_table = arc_lengths
        self._param_table = t_vals
        self._computed_total_length = arc_lengths[-1]

    def evaluate_arc_length(self, s: float, derivative: int = 0) -> np.ndarray:
        """
        Evaluate the spline at a given arc length using accurate parameterization.

        Args:
            s: Arc length value (0.0 to total_length)
            derivative: Derivative order (0=position, 1=velocity, 2=acceleration)

        Returns:
            3D point or derivative vector at arc length s
        """
        # Compute arc length table if not done yet
        self._compute_arc_length_table()

        # Clamp s to valid range
        s = np.clip(s, 0.0, self._computed_total_length)

        # Find parameter t corresponding to arc length s
        if s <= 0:
            t = 0.0
        elif s >= self._computed_total_length:
            t = 1.0
        else:
            # Interpolate to find parameter t
            t = np.interp(s, self._arc_length_table, self._param_table)

        return self.evaluate(t, derivative)

    @property
    def total_length(self) -> float:
        """Get the total arc length of the spline."""
        self._compute_arc_length_table()
        return self._computed_total_length

    def get_frenet_frame(self, t: float) -> dict:
        """
        Get the Frenet frame at a given parameter value.

        Args:
            t: Parameter value (0.0 to 1.0)

        Returns:
            Dictionary with 'position', 'tangent', and 'normal' vectors
        """
        position = self.evaluate(t, derivative=0)
        tangent = self.evaluate(t, derivative=1)

        # Normalize tangent
        tangent_norm = np.linalg.norm(tangent)
        if tangent_norm > 0:
            tangent = tangent / tangent_norm
        else:
            tangent = np.array([1.0, 0.0, 0.0])

        # Calculate normal (assuming 2D in XY plane)
        # For 2D: normal is perpendicular to tangent in XY plane
        normal = np.array([-tangent[1], tangent[0], 0.0])
        normal_norm = np.linalg.norm(normal)
        if normal_norm > 0:
            normal = normal / normal_norm
        else:
            normal = np.array([0.0, 1.0, 0.0])

        return {"position": position, "tangent": tangent, "normal": normal}
