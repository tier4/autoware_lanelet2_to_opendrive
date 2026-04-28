"""Border polynomial fitting for OpenDRIVE <lane><border> emission.

Projects a lane's outer boundary onto a road reference line and fits a
piecewise-cubic ``t(s)`` polynomial. Optional hard endpoint constraints
pin ``t(0)`` and/or ``t(total_length)`` so that connecting-road lane
edges line up with the linked regular roads' lane edges (see
docs/superpowers/specs/2026-04-28-issue-437-pin-junction-endpoints-default-on-design.md).
"""

from __future__ import annotations

import logging
from typing import List, Optional

import numpy as np

from .cubic_spline_1d import CubicSpline1D
from .opendrive.lane_elements import LaneBorder
from .spline import Splines

logger = logging.getLogger(__name__)


def fit_lane_border_polynomials(
    boundary_points_3d: np.ndarray,
    reference_line_spline: Splines,
    t_start_constraint: Optional[float] = None,
    t_end_constraint: Optional[float] = None,
) -> List[LaneBorder]:
    """Fit a piecewise-cubic border t(s) over the road's s-range.

    Args:
        boundary_points_3d: Lane outer boundary as a (N, 3) array. Only the
            (x, y) components are used for projection; z is ignored.
        reference_line_spline: The (already-pinned) road reference line.
        t_start_constraint: If set, force ``t(0) = t_start_constraint``
            exactly by injecting a data point at ``s=0``.
        t_end_constraint: If set, force ``t(total_length) =
            t_end_constraint`` exactly by injecting a data point at
            ``s=total_length``.

    Returns:
        A list of :class:`LaneBorder` segments aligned with the underlying
        cubic spline's natural knots. Each segment encodes one piecewise
        polynomial in OpenDRIVE coefficient form
        ``t(s) = a + b·(s - s_offset) + c·(s - s_offset)^2 + d·(s - s_offset)^3``.
    """
    if boundary_points_3d.ndim != 2 or boundary_points_3d.shape[1] != 3:
        raise ValueError(
            f"boundary_points_3d must have shape (N, 3), got {boundary_points_3d.shape}"
        )
    if len(boundary_points_3d) < 2:
        raise ValueError("boundary_points_3d must contain at least 2 points")

    L = reference_line_spline.total_length

    # Step 1: project each boundary point onto the reference line.
    s_list: List[float] = []
    t_list: List[float] = []
    for i in range(len(boundary_points_3d)):
        p = np.asarray(boundary_points_3d[i, :2], dtype=float)
        s = reference_line_spline.find_closest_s(p)
        ref_xy = reference_line_spline.evaluate(s)[:2]
        d_xy = reference_line_spline.evaluate(s, derivative=1)[:2]
        norm = float(np.linalg.norm(d_xy))
        if norm < 1e-12:
            logger.warning(
                "Reference line tangent is degenerate at s=%.3f; skipping point %d",
                s,
                i,
            )
            continue
        tangent = d_xy / norm
        # Left-handed +90° rotation: positive t means "left of reference"
        normal = np.array([-tangent[1], tangent[0]])
        t = float(np.dot(p - ref_xy, normal))
        s_list.append(float(s))
        t_list.append(t)

    if len(s_list) < 2:
        raise ValueError(
            "Fewer than 2 boundary points survived projection — cannot fit border"
        )

    # Step 2: sort by s and drop strictly-monotonic violations.
    order = np.argsort(np.asarray(s_list))
    s_arr = np.asarray(s_list)[order]
    t_arr = np.asarray(t_list)[order]

    keep_mask = np.concatenate(([True], np.diff(s_arr) > 1e-6))
    if not np.all(keep_mask):
        logger.debug(
            "Dropped %d non-monotonic projected samples", int(np.sum(~keep_mask))
        )
    s_arr = s_arr[keep_mask]
    t_arr = t_arr[keep_mask]

    # Step 3: inject endpoint constraints.
    if t_start_constraint is not None:
        if abs(s_arr[0]) < 1e-6:
            t_arr = t_arr.copy()
            t_arr[0] = float(t_start_constraint)
        else:
            s_arr = np.concatenate(([0.0], s_arr))
            t_arr = np.concatenate(([float(t_start_constraint)], t_arr))

    if t_end_constraint is not None:
        if abs(s_arr[-1] - L) < 1e-6:
            t_arr = t_arr.copy()
            t_arr[-1] = float(t_end_constraint)
        else:
            s_arr = np.concatenate((s_arr, [L]))
            t_arr = np.concatenate((t_arr, [float(t_end_constraint)]))

    # Step 4: fit cubic spline, convert to LaneBorder segments.
    if len(s_arr) < 2:
        raise ValueError(
            "Fewer than 2 spline knots after constraint injection — cannot fit border"
        )

    spline_1d = CubicSpline1D(s_arr, t_arr, bc_type="not-a-knot")

    return [
        LaneBorder(
            s_offset=float(s_off), a=float(a), b=float(b), c=float(c), d=float(d)
        )
        for (s_off, a, b, c, d) in spline_1d.get_segments()
    ]
