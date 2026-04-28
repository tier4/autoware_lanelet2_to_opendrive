"""B-Spline interpolation with constrained fitting."""

import numpy as np
import warnings
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
from scipy.interpolate import BSpline
from scipy.optimize import minimize_scalar

from .config import DEFAULT_CONFIG


def compute_dynamic_control_points(points: np.ndarray) -> int:
    """
    Compute the optimal number of control points based on input geometry.

    This function analyzes the input points to determine an appropriate number
    of control points for spline fitting, considering:
    1. Total number of input points
    2. Local curvature characteristics
    3. Geometric complexity

    Args:
        points: (N, 2) or (N, 3) Array of input points

    Returns:
        Optimal number of control points (clamped to min/max bounds)

    Algorithm:
        1. Base calculation: num_points * control_points_ratio
        2. Curvature adjustment: Increase control points in high-curvature regions
        3. Clamp to [min_control_points, max_control_points] range
    """
    points = np.asarray(points)
    n_points = len(points)

    # Step 1: Base calculation from input points
    base_control_points = int(n_points * DEFAULT_CONFIG.spline.control_points_ratio)

    # Step 2: Analyze curvature to adjust control points
    if n_points >= 3:
        curvature_factor = _analyze_curvature_complexity(points)
        adjusted_control_points = int(base_control_points * curvature_factor)
    else:
        adjusted_control_points = base_control_points

    # Step 3: Clamp to valid range
    final_control_points = np.clip(
        adjusted_control_points,
        DEFAULT_CONFIG.spline.min_control_points,
        DEFAULT_CONFIG.spline.max_control_points,
    )

    return final_control_points


def _analyze_curvature_complexity(points: np.ndarray) -> float:
    """
    Analyze curvature complexity to determine control point multiplier.

    Calculates angles between adjacent tangent vectors to estimate curvature.
    High-curvature regions require more control points for accurate fitting.

    Args:
        points: (N, 2) or (N, 3) Array of input points (N >= 3)

    Returns:
        Curvature factor in range [1.0, curvature_multiplier]
        - 1.0 for low curvature (straight lines)
        - curvature_multiplier for high curvature (sharp turns)
    """
    # Calculate tangent vectors for each segment
    diffs = np.diff(points, axis=0)
    norms = np.linalg.norm(diffs, axis=1)

    # Avoid division by zero
    norms[norms == 0] = 1.0
    tangents = diffs / norms[:, None]

    # Calculate angles between adjacent tangents (approximation of curvature)
    if len(tangents) < 2:
        return 1.0

    dot_products = np.sum(tangents[:-1] * tangents[1:], axis=1)
    dot_products = np.clip(dot_products, -1.0, 1.0)
    angles = np.arccos(dot_products)

    # Calculate statistics of curvature distribution
    mean_angle = np.mean(angles)
    max_angle = np.max(angles)

    # Determine curvature factor based on thresholds
    threshold = DEFAULT_CONFIG.spline.curvature_threshold
    multiplier = DEFAULT_CONFIG.spline.curvature_multiplier

    # If mean curvature is high, increase control points significantly
    if mean_angle > threshold:
        return multiplier

    # If max curvature is high, increase control points moderately
    if max_angle > threshold * 2:
        return (multiplier + 1.0) / 2.0

    # Otherwise, use base control points
    return 1.0


@dataclass
class DesignMatrices:
    """Design matrices for constrained spline fitting.

    Attributes:
        basis_data: Basis functions evaluated at data points (N x num_control_points)
        basis_start_vel: Basis function derivatives at start (1 x num_control_points)
        basis_end_vel: Basis function derivatives at end (1 x num_control_points)
        target_data: Target data points (N x 3)
        target_start_vel: Target start velocity (1 x 3)
        target_end_vel: Target end velocity (1 x 3)
        weights: Constraint weights (hard, soft)
    """

    basis_data: np.ndarray
    basis_start_vel: np.ndarray
    basis_end_vel: np.ndarray
    target_data: np.ndarray
    target_start_vel: np.ndarray
    target_end_vel: np.ndarray
    weights: Dict[str, float]


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
        num_control_points: Optional[int] = None,
        k: int = 3,
        hard_constraint_weight: Optional[float] = None,
    ):
        """
        Initialize a B-spline with constrained fitting.

        Args:
            points: (N, 2) or (N, 3) Array of 2D or 3D points to fit
            start_vel: (2,) or (3,) Tangent vector at the start. If None, estimated from points.
            end_vel: (2,) or (3,) Tangent vector at the end. If None, estimated from points.
            num_control_points: Number of control points (higher = closer fit, lower = smoother).
                               If None, automatically computed based on input geometry.
            k: Degree of the spline (usually 3 for cubic)
            hard_constraint_weight: Override for the weight applied to the
                boundary position and velocity constraints when solving the
                least-squares problem.  When the caller needs the fit to
                land on the first/last input point to higher precision than
                the default mix allows (e.g. for junction endpoint fidelity
                where the first/last point has been overridden to the
                neighbour road's endpoint), pass a larger value such as
                ``1e4`` (sub-millimetre endpoint accuracy while keeping the
                interior fit close to the default).  Avoid extremely large
                values such as ``1e6``: they distort the interior enough
                that the rendered reference line can drift outside the
                corridor of the source lanelets and confuse the
                lanelet-to-road geometric matcher.  If ``None``, the
                configured default is used.

        Raises:
            ValueError: If too few control points or invalid input
        """
        self.points = np.asarray(points)
        if self.points.ndim != 2 or self.points.shape[1] not in [2, 3]:
            raise ValueError("Points must be an (N, 2) or (N, 3) array")

        # Convert 2D points to 3D by adding z=0
        if self.points.shape[1] == 2:
            self.points = np.column_stack([self.points, np.zeros(len(self.points))])

        self.n_points = len(self.points)
        if self.n_points < 2:
            raise ValueError("At least 2 points are required")

        # Store original coordinate offset for numerical stability
        self._origin_offset = self.points[0].copy()

        # Translate points to origin for numerical stability with large coordinates
        self.points = self.points - self._origin_offset

        self.k = k

        # Automatically compute control points if not specified
        if num_control_points is None:
            self.num_control_points = compute_dynamic_control_points(self.points)
        else:
            self.num_control_points = num_control_points

        # Estimate tangent vectors if not provided
        if start_vel is None:
            if self.n_points >= 3:
                # Use second-order estimation for better accuracy
                start_vel = self.points[1] - self.points[0]
                start_norm = np.linalg.norm(start_vel)
                if start_norm > DEFAULT_CONFIG.geometry.epsilon:
                    start_vel = start_vel / start_norm
                else:
                    # Fall back to longer distance if points are duplicate
                    start_vel = self.points[2] - self.points[0]
                    start_norm = np.linalg.norm(start_vel)
                    if start_norm > DEFAULT_CONFIG.geometry.epsilon:
                        start_vel = start_vel / start_norm
                    else:
                        start_vel = np.array([1.0, 0.0, 0.0])
            elif self.n_points >= 2:
                start_vel = self.points[1] - self.points[0]
                start_norm = np.linalg.norm(start_vel)
                if start_norm > DEFAULT_CONFIG.geometry.epsilon:
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
                if end_norm > DEFAULT_CONFIG.geometry.epsilon:
                    end_vel = end_vel / end_norm
                else:
                    # Fall back to longer distance if points are duplicate
                    end_vel = self.points[-1] - self.points[-3]
                    end_norm = np.linalg.norm(end_vel)
                    if end_norm > DEFAULT_CONFIG.geometry.epsilon:
                        end_vel = end_vel / end_norm
                    else:
                        end_vel = np.array([1.0, 0.0, 0.0])
            elif self.n_points >= 2:
                end_vel = self.points[-1] - self.points[-2]
                end_norm = np.linalg.norm(end_vel)
                if end_norm > DEFAULT_CONFIG.geometry.epsilon:
                    end_vel = end_vel / end_norm
                else:
                    end_vel = np.array([1.0, 0.0, 0.0])
            else:
                end_vel = np.array([1.0, 0.0, 0.0])

        self.start_vel = np.asarray(start_vel)
        self.end_vel = np.asarray(end_vel)

        # Convert 2D velocity vectors to 3D by adding z=0
        if self.start_vel.shape[0] == 2:
            self.start_vel = np.append(self.start_vel, 0.0)
        if self.end_vel.shape[0] == 2:
            self.end_vel = np.append(self.end_vel, 0.0)

        # Allow callers (e.g. junction endpoint override path) to raise the
        # hard-constraint weight so boundary conditions are honoured to
        # near machine precision.
        self._hard_constraint_weight: Optional[float] = hard_constraint_weight

        # Perform constrained spline fitting
        self._fit_constrained_spline()

    def _fit_constrained_spline(self) -> None:
        """
        Fit constrained B-spline through points with boundary conditions.

        High-level orchestration of spline fitting pipeline:
        1. Create parameterization (chord-length)
        2. Create knot vector (curvature-adaptive)
        3. Build design matrices for constraints
        4. Solve constrained least squares problem
        """
        # Step 1: Create parameterization
        self.t_data, self.t_max = self._create_parameterization(self.points)

        # Step 2: Create knot vector
        self.knots = self._create_knot_vector(self.t_data, self.points)

        # Step 3: Build design matrices
        matrices = self._build_design_matrices(
            self.t_data, self.knots, self.points, self.start_vel, self.end_vel
        )

        # Step 4: Solve least squares problem
        self.coeffs = self._solve_constrained_least_squares(matrices)

        # Create the B-Spline object
        self.spline = BSpline(self.knots, self.coeffs, self.k)

        # Verify hard constraints are satisfied
        self._verify_hard_constraints()

        # Check soft constraints and warn if fitting error is large
        self._check_soft_constraints()

    def _create_parameterization(self, points: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Create chord-length parameterization for input points.

        Args:
            points: Nx3 array of data points

        Returns:
            Tuple of (t_values, t_max) where:
                t_values: Parameter values t_i in [0, 1]
                t_max: Maximum parameter value before normalization
        """
        # Compute cumulative chord lengths
        dists = np.linalg.norm(np.diff(points, axis=0), axis=1)
        t_data = np.concatenate(([0], np.cumsum(dists)))
        t_max = t_data[-1]

        # Handle zero length case
        if t_max == 0:
            t_max = 1.0
            t_data = np.linspace(0, 1, len(points))
        else:
            t_data /= t_max  # Normalize to 0.0 ~ 1.0

        return t_data, t_max

    def _compute_curvature_weights(self, points: np.ndarray) -> np.ndarray:
        """
        Compute curvature-based weights for adaptive knot placement.

        Calculates angles between adjacent tangent vectors as an approximation
        of curvature. Higher angles (sharper turns) receive higher weights.

        Args:
            points: Nx3 array of data points

        Returns:
            Weights for each point based on local curvature (length N)
        """
        # A. Calculate tangent vectors for each segment
        diffs = np.diff(points, axis=0)
        norms = np.linalg.norm(diffs, axis=1)
        # Avoid division by zero
        norms[norms == 0] = 1.0
        tangents = diffs / norms[:, None]

        # B. Calculate angles between adjacent tangents (approximation of curvature)
        # Compute dot products to get angles (N-2 angles)
        dot_products = np.sum(tangents[:-1] * tangents[1:], axis=1)
        # Clip for numerical stability
        dot_products = np.clip(dot_products, -1.0, 1.0)
        angles = np.arccos(dot_products)

        # C. Create weight distribution
        # Endpoints have no angle, so set to 0
        curvature_metric = np.concatenate(([0], angles, [0]))

        # Density function D(t) for knot placement
        # alpha: weight for uniformity (larger values approach uniform spacing)
        # beta:  weight for curvature (larger values concentrate knots at curves)
        alpha = DEFAULT_CONFIG.spline.knot_alpha_weight
        beta = DEFAULT_CONFIG.spline.knot_beta_weight

        weights = alpha + beta * curvature_metric

        return weights

    def _create_knot_vector(
        self, t_values: np.ndarray, points: np.ndarray
    ) -> np.ndarray:
        """
        Create curvature-adaptive knot vector for B-spline.

        Uses inverse transform sampling to place knots based on curvature:
        more knots are placed in high-curvature regions.

        Args:
            t_values: Parameter values for data points (length N)
            points: Original data points for curvature calculation (Nx3)

        Returns:
            Knot vector for B-spline basis (includes endpoint multiplicities)
        """
        n_internal = self.num_control_points - (self.k + 1)
        if n_internal < 0:
            raise ValueError(f"Too few control points. Must be at least {self.k + 1}.")

        if n_internal > 0:
            # Compute curvature-based weights
            weights = self._compute_curvature_weights(points)

            # Create Cumulative Distribution Function (CDF)
            cdf = np.cumsum(weights)
            cdf_normalized = cdf / cdf[-1]

            # Select knots uniformly in CDF space and map back to t-space
            # (Inverse Transform Sampling)
            target_knots_cdf = np.linspace(0, 1, n_internal + 2)[1:-1]
            internal_knots = np.interp(target_knots_cdf, cdf_normalized, t_values)
        else:
            internal_knots = []

        # Create full knot vector with endpoint multiplicities
        knots = np.concatenate(
            [
                np.zeros(self.k + 1),
                internal_knots,
                np.ones(self.k + 1),
            ]
        )

        return knots

    def _build_design_matrices(
        self,
        t_values: np.ndarray,
        knot_vector: np.ndarray,
        points: np.ndarray,
        start_vel: np.ndarray,
        end_vel: np.ndarray,
    ) -> DesignMatrices:
        """
        Build design matrices for constrained least squares.

        Creates basis function matrices for data fitting (soft constraints)
        and boundary conditions (hard constraints).

        Args:
            t_values: Parameter values for data points
            knot_vector: Knot vector for basis functions
            points: Data points to fit
            start_vel: Desired start velocity
            end_vel: Desired end velocity

        Returns:
            DesignMatrices containing all components for solving
        """
        # Data fitting term (Soft constraint)
        A_fit = self._get_basis_matrix(t_values, deriv=0)
        b_fit = points

        # Boundary condition terms (Hard constraints)
        # Derivatives (Tangent vectors) at t=0, t=1
        A_vel_start = self._get_basis_matrix([0.0], deriv=1)
        A_vel_end = self._get_basis_matrix([1.0], deriv=1)

        # Get weights from config (or per-instance override for callers that
        # need boundary constraints enforced to near machine precision).
        w_hard = (
            self._hard_constraint_weight
            if self._hard_constraint_weight is not None
            else DEFAULT_CONFIG.spline.hard_constraint_weight
        )
        w_soft = DEFAULT_CONFIG.spline.soft_constraint_weight

        # Package everything into DesignMatrices dataclass
        matrices = DesignMatrices(
            basis_data=A_fit,
            basis_start_vel=A_vel_start,
            basis_end_vel=A_vel_end,
            target_data=b_fit,
            target_start_vel=start_vel,
            target_end_vel=end_vel,
            weights={"hard": w_hard, "soft": w_soft},
        )

        return matrices

    def _solve_constrained_least_squares(self, matrices: DesignMatrices) -> np.ndarray:
        """
        Solve weighted least squares with hard and soft constraints.

        Combines data fitting constraints (soft) and boundary conditions (hard)
        into a single weighted least squares problem.

        Args:
            matrices: Pre-assembled design matrices and targets

        Returns:
            Optimal control points (num_control_points x 3)
        """
        w_hard = matrices.weights["hard"]
        w_soft = matrices.weights["soft"]

        # Get boundary position constraints
        # We need to get the actual start/end positions from target_data
        # since DesignMatrices doesn't store them separately
        start_pos = matrices.target_data[[0]]
        end_pos = matrices.target_data[[-1]]

        # Build basis matrices for position constraints
        A_pos_start = self._get_basis_matrix([0.0], deriv=0)
        A_pos_end = self._get_basis_matrix([1.0], deriv=0)

        # Combine matrices with weights
        A_combined = np.vstack(
            [
                matrices.basis_data * w_soft,
                A_pos_start * w_hard,
                A_pos_end * w_hard,
                matrices.basis_start_vel * w_hard,
                matrices.basis_end_vel * w_hard,
            ]
        )

        # Combine targets (scale velocities by t_max for normalized parameter space)
        b_combined = np.vstack(
            [
                matrices.target_data * w_soft,
                start_pos * w_hard,
                end_pos * w_hard,
                (matrices.target_start_vel * self.t_max * w_hard).reshape(1, -1),
                (matrices.target_end_vel * self.t_max * w_hard).reshape(1, -1),
            ]
        )

        # Solve for control points (Least Squares)
        coeffs_x, _, _, _ = np.linalg.lstsq(A_combined, b_combined[:, 0], rcond=None)
        coeffs_y, _, _, _ = np.linalg.lstsq(A_combined, b_combined[:, 1], rcond=None)
        coeffs_z, _, _, _ = np.linalg.lstsq(A_combined, b_combined[:, 2], rcond=None)

        coeffs = np.column_stack([coeffs_x, coeffs_y, coeffs_z])

        return coeffs

    def _verify_hard_constraints(
        self,
        position_tol: Optional[float] = None,
        velocity_tol: Optional[float] = None,
    ):
        """
        Verify that hard constraints (boundary conditions) are satisfied.

        Args:
            position_tol: Tolerance for position constraints
            velocity_tol: Tolerance for velocity constraints

        Raises:
            ValueError: If any hard constraint is violated beyond tolerance
        """
        if position_tol is None:
            position_tol = DEFAULT_CONFIG.spline.position_tolerance
        if velocity_tol is None:
            velocity_tol = DEFAULT_CONFIG.spline.velocity_tolerance

        # Check start position constraint
        start_pos_actual = self.spline(0.0, nu=0)
        start_pos_expected = self.points[0]
        start_pos_error = np.linalg.norm(start_pos_actual - start_pos_expected)
        if start_pos_error > position_tol:
            raise ValueError(
                f"Start position constraint violated: "
                f"error={start_pos_error:.6f} > tolerance={position_tol:.6f}\n"
                f"Expected: {start_pos_expected}, Got: {start_pos_actual}"
            )

        # Check end position constraint
        end_pos_actual = self.spline(1.0, nu=0)
        end_pos_expected = self.points[-1]
        end_pos_error = np.linalg.norm(end_pos_actual - end_pos_expected)
        if end_pos_error > position_tol:
            raise ValueError(
                f"End position constraint violated: "
                f"error={end_pos_error:.6f} > tolerance={position_tol:.6f}\n"
                f"Expected: {end_pos_expected}, Got: {end_pos_actual}"
            )

        # Check start velocity constraint
        # Note: velocities in normalized parameter space need to be scaled
        start_vel_actual = self.spline(0.0, nu=1) / self.t_max
        start_vel_expected = self.start_vel
        start_vel_error = np.linalg.norm(start_vel_actual - start_vel_expected)
        if start_vel_error > velocity_tol:
            raise ValueError(
                f"Start velocity constraint violated: "
                f"error={start_vel_error:.6f} > tolerance={velocity_tol:.6f}\n"
                f"Expected direction: {start_vel_expected}, Got: {start_vel_actual}"
            )

        # Check end velocity constraint
        end_vel_actual = self.spline(1.0, nu=1) / self.t_max
        end_vel_expected = self.end_vel
        end_vel_error = np.linalg.norm(end_vel_actual - end_vel_expected)
        if end_vel_error > velocity_tol:
            raise ValueError(
                f"End velocity constraint violated: "
                f"error={end_vel_error:.6f} > tolerance={velocity_tol:.6f}\n"
                f"Expected direction: {end_vel_expected}, Got: {end_vel_actual}"
            )

    def _check_soft_constraints(
        self,
        max_avg_error: Optional[float] = None,
        max_point_error: Optional[float] = None,
        warn_percentile: Optional[float] = None,
    ):
        """
        Check soft constraints (data fitting) and warn if error is large.

        Args:
            max_avg_error: Maximum average error before warning (in coordinate units)
            max_point_error: Maximum single point error before warning
            warn_percentile: Percentile of errors to report in warning

        Warns:
            UserWarning if fitting errors exceed thresholds
        """
        if max_avg_error is None:
            max_avg_error = DEFAULT_CONFIG.spline.max_avg_error
        if max_point_error is None:
            max_point_error = DEFAULT_CONFIG.spline.max_point_error
        if warn_percentile is None:
            warn_percentile = DEFAULT_CONFIG.spline.warn_percentile

        # Calculate errors at each input point
        errors = []
        for i, t in enumerate(self.t_data):
            fitted_pos = self.spline(t, nu=0)
            actual_pos = self.points[i]
            error = np.linalg.norm(fitted_pos - actual_pos)
            errors.append(error)

        errors = np.array(errors)
        avg_error = np.mean(errors)
        max_error = np.max(errors)
        max_error_idx = np.argmax(errors)
        percentile_error = np.percentile(errors, warn_percentile)

        # Prepare warning messages
        warnings_list = []

        if avg_error > max_avg_error:
            warnings_list.append(
                f"Average fitting error ({avg_error:.3f}) exceeds threshold ({max_avg_error:.3f})"
            )

        if max_error > max_point_error:
            warnings_list.append(
                f"Maximum fitting error ({max_error:.3f}) at point {max_error_idx} "
                f"exceeds threshold ({max_point_error:.3f})"
            )

        # Issue combined warning if any threshold is exceeded
        if warnings_list:
            warning_msg = (
                f"Spline fitting quality warning:\n"
                f"  {'  '.join(warnings_list)}\n"
                f"  Statistics: avg={avg_error:.3f}, max={max_error:.3f}, "
                f"  {warn_percentile:.0f}th percentile={percentile_error:.3f}\n"
                f"  Consider increasing num_control_points or adjusting w_soft weight"
            )
            warnings.warn(warning_msg, UserWarning, stacklevel=3)

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

    def evaluate(self, s: float, derivative: int = 0) -> np.ndarray:
        """
        Evaluate the spline at a given arc length in Frenet coordinate system.

        Args:
            s: Arc length value from the first point (0.0 to total_length)
            derivative: Derivative order (0=position, 1=velocity, 2=acceleration)

        Returns:
            3D point or derivative vector at arc length s
        """
        # Convert arc length to normalized parameter t
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

        if derivative == 0:
            # For position, translate back to original coordinate system
            result = self.spline(t, nu=0)
            return result + self._origin_offset
        else:
            # For derivatives with respect to arc length, we need to apply the chain rule
            # ds/dt = ||dx/dt|| where x is position
            # d/ds = (d/dt) * (dt/ds) = (d/dt) / (ds/dt)

            velocity_t = self.spline(t, nu=1)  # dx/dt
            speed = np.linalg.norm(velocity_t)  # ds/dt = ||dx/dt||

            if derivative == 1:
                # dx/ds = (dx/dt) / (ds/dt)
                if speed > DEFAULT_CONFIG.spline.speed_epsilon:
                    return velocity_t / speed
                else:
                    return np.array([1.0, 0.0, 0.0])  # Fallback direction
            elif derivative == 2:
                # For second derivative: d²x/ds²
                if speed > DEFAULT_CONFIG.spline.speed_epsilon:
                    accel_t = self.spline(t, nu=2)  # d²x/dt²
                    # Apply chain rule for second derivative
                    # d²x/ds² = (d²x/dt²) / (ds/dt)² - (dx/dt) * (d²s/dt²) / (ds/dt)³
                    # where d²s/dt² = (dx/dt · d²x/dt²) / ||dx/dt||
                    dsdt_squared_derivative = np.dot(velocity_t, accel_t) / speed
                    return accel_t / (
                        speed * speed
                    ) - velocity_t * dsdt_squared_derivative / (speed * speed * speed)
                else:
                    return np.array([0.0, 0.0, 0.0])
            else:
                # Higher order derivatives not implemented
                raise NotImplementedError(
                    f"Derivative order {derivative} not supported"
                )

    def _evaluate_normalized(self, t: float, derivative: int = 0) -> np.ndarray:
        """
        Evaluate the spline at a given normalized parameter value.

        Args:
            t: Normalized parameter value (0.0 to 1.0)
            derivative: Derivative order (0=position, 1=velocity, 2=acceleration)

        Returns:
            3D point or derivative vector at parameter t
        """
        return self.spline(t, nu=derivative)

    def _compute_arc_length_table(
        self, num_samples: int = DEFAULT_CONFIG.spline.arc_length_table_samples
    ) -> None:
        """Compute arc length lookup table for accurate parameterization."""
        if hasattr(self, "_arc_length_table"):
            return

        t_vals = np.linspace(0.0, 1.0, num_samples)
        positions = np.array([self._evaluate_normalized(t) for t in t_vals])

        # Compute arc lengths
        distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        arc_lengths = np.concatenate(([0], np.cumsum(distances)))

        self._arc_length_table = arc_lengths
        self._param_table = t_vals
        self._computed_total_length = arc_lengths[-1]

    def evaluate_arc_length(self, s: float, derivative: int = 0) -> np.ndarray:
        """
        Evaluate the spline at a given arc length (alias for evaluate method).

        Args:
            s: Arc length value (0.0 to total_length)
            derivative: Derivative order (0=position, 1=velocity, 2=acceleration)

        Returns:
            3D point or derivative vector at arc length s
        """
        return self.evaluate(s, derivative)

    @property
    def total_length(self) -> float:
        """Get the total arc length of the spline."""
        self._compute_arc_length_table()
        return self._computed_total_length

    def cartesian_to_frenet(
        self, x: float, y: float, z: float = 0.0
    ) -> Tuple[float, float]:
        """
        Convert Cartesian coordinates (x, y, z) to Frenet coordinates (s, d).

        This method finds the closest point on the spline to the query point
        and returns the arc length (s) and lateral offset (d) in the Frenet frame.

        Args:
            x: X coordinate of the query point
            y: Y coordinate of the query point
            z: Z coordinate of the query point (default: 0.0)

        Returns:
            Tuple of (s, d) where:
                s: Arc length from the start of the spline to the closest point
                d: Lateral offset from the spline (positive = left, negative = right)
                   Left/right is determined based on the tangent direction in XY plane

        Note:
            - The lateral offset sign is determined by the 2D cross product in the XY plane
            - For 2D applications, set z=0.0 (default)
            - The method uses numerical optimization to find the closest point
        """
        # Ensure arc length table is computed
        self._compute_arc_length_table()

        # Query point in original coordinate system
        query_point = np.array([x, y, z])

        # Translate to spline's local coordinate system
        query_point_local = query_point - self._origin_offset

        # Step 1: Find the parameter t that minimizes distance to the query point
        t_closest = self._find_closest_parameter(query_point_local)

        # Step 2: Convert parameter t to arc length s
        s = np.interp(t_closest, self._param_table, self._arc_length_table)

        # Step 3: Calculate lateral offset d
        d = self._calculate_lateral_offset(query_point_local, t_closest)

        return s, d

    def _find_closest_parameter(self, query_point_local: np.ndarray) -> float:
        """
        Find the parameter t that minimizes distance to the query point.

        Args:
            query_point_local: Query point in spline's local coordinate system

        Returns:
            Parameter t in [0, 1] that gives the closest point on spline
        """
        # Use bounded optimization to find closest point on spline
        result = minimize_scalar(
            lambda t: self._distance_squared_at_parameter(t, query_point_local),
            bounds=(0.0, 1.0),
            method="bounded",
        )
        return result.x

    def _distance_squared_at_parameter(
        self, t: float, query_point_local: np.ndarray
    ) -> float:
        """
        Calculate squared distance from spline to query point at parameter t.

        Args:
            t: Parameter value in [0, 1]
            query_point_local: Query point in spline's local coordinate system

        Returns:
            Squared distance from spline point to query point
        """
        t = np.clip(t, 0.0, 1.0)  # Ensure t is within valid range
        spline_point = self._evaluate_normalized(t, derivative=0)
        diff = spline_point - query_point_local
        return np.dot(diff, diff)

    def _calculate_lateral_offset(
        self, query_point_local: np.ndarray, t_closest: float
    ) -> float:
        """
        Calculate lateral offset from spline to query point.

        Args:
            query_point_local: Query point in spline's local coordinate system
            t_closest: Parameter t of closest point on spline

        Returns:
            Lateral offset d (positive = left, negative = right)
        """
        # Get the closest point on the spline
        closest_point_local = self._evaluate_normalized(t_closest, derivative=0)

        # Vector from closest point to query point (in local coordinates)
        vec_to_query = query_point_local - closest_point_local

        # Calculate absolute distance in XY plane only
        # (z-component should not affect lateral offset)
        d_abs = np.linalg.norm(vec_to_query[:2])

        # Determine sign based on tangent direction (2D cross product in XY plane)
        tangent_t = self._evaluate_normalized(t_closest, derivative=1)

        # 2D cross product: tangent × vec_to_query (only consider XY plane)
        # If positive, point is on the left; if negative, on the right
        cross_product = tangent_t[0] * vec_to_query[1] - tangent_t[1] * vec_to_query[0]

        # Assign sign: positive for left, negative for right
        return d_abs if cross_product >= 0 else -d_abs
