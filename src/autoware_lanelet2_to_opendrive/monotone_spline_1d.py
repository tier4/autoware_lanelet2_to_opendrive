"""Monotone cubic spline (PCHIP) for 1D interpolation."""

from typing import Tuple

import numpy as np
from scipy.interpolate import PchipInterpolator

from .spline_1d_base import Spline1DBase


class MonotoneSpline1D(Spline1DBase):
    """
    Monotone cubic spline using PCHIP (Piecewise Cubic Hermite Interpolating Polynomial).

    This spline guarantees:
    - No overshoot: interpolated values stay within input range
    - Shape preservation: monotonicity is preserved
    - Positive values: if input values are positive, output values are also positive
    - C1 continuity: function and first derivative are continuous

    This is ideal for width interpolation where negative values must be avoided.
    """

    def __init__(self, arc_lengths: np.ndarray, values: np.ndarray):
        """
        Initialize a monotone 1D spline using PCHIP.

        Args:
            arc_lengths: Array of arc length values (must be monotonically increasing)
            values: Array of values corresponding to arc_lengths
        """
        super().__init__(arc_lengths, values)

        # Create PCHIP interpolator
        self.spline = PchipInterpolator(self.arc_lengths, self.values)

    def evaluate(self, s: float, derivative: int = 0) -> float:
        """
        Evaluate the spline at a given arc length.

        Args:
            s: Arc length value
            derivative: Derivative order (0=value, 1=first derivative, 2=second derivative)

        Returns:
            Value or derivative at arc length s
        """
        # Clamp to valid range
        s = np.clip(s, 0, self.total_length)

        # PCHIP supports derivatives up to order 2
        if derivative > 2:
            raise ValueError(
                f"PCHIP only supports derivatives up to order 2, got {derivative}"
            )

        return float(self.spline(s, derivative))

    def get_polynomial_coefficients(
        self, segment_index: int
    ) -> Tuple[float, float, float, float]:
        """
        Get polynomial coefficients (a, b, c, d) for a specific segment.

        The polynomial is: f(s) = a + b*(s-s0) + c*(s-s0)^2 + d*(s-s0)^3
        where s0 is the start of the segment.

        Args:
            segment_index: Index of the segment (0 to num_segments-1)

        Returns:
            Tuple of (a, b, c, d) coefficients

        Raises:
            ValueError: If segment_index is out of range
        """
        if segment_index < 0 or segment_index >= self.num_segments:
            raise ValueError(
                f"Invalid segment index: {segment_index}. "
                f"Must be in range [0, {self.num_segments-1}]"
            )

        # PCHIP stores coefficients in the same format as scipy's CubicSpline
        # Access the internal coefficient matrix
        # self.spline.c has shape (4, n-1) where n is number of knots
        # Each column represents [d, c, b, a] for that segment (note the reverse order)

        if hasattr(self.spline, "c"):
            c_matrix = self.spline.c
            if segment_index < c_matrix.shape[1]:
                # The coefficients are stored in reverse order
                d = c_matrix[0, segment_index]  # Coefficient of (s-s0)^3
                c = c_matrix[1, segment_index]  # Coefficient of (s-s0)^2
                b = c_matrix[2, segment_index]  # Coefficient of (s-s0)
                a = c_matrix[3, segment_index]  # Constant term

                return float(a), float(b), float(c), float(d)

        # Fallback: compute from derivatives (less efficient but always works)
        s0 = self.segment_starts[segment_index]
        a = self.spline(s0, 0)  # f(s0)
        b = self.spline(s0, 1)  # f'(s0)
        c = self.spline(s0, 2) / 2.0  # f''(s0) / 2!

        # For PCHIP, we need to estimate d from the next point
        # This is less accurate than using the internal representation
        s1 = self.segment_ends[segment_index]
        ds = s1 - s0

        # Solve for d using the constraint: f(s1) = a + b*ds + c*ds^2 + d*ds^3
        f_s1 = self.spline(s1, 0)
        d = (f_s1 - a - b * ds - c * ds**2) / (ds**3)

        return float(a), float(b), float(c), float(d)
