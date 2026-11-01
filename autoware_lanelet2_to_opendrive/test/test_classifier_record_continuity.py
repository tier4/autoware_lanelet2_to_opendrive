"""The emitted <geometry> records must form one unbroken reference line.

A reader stitches a road's plan view by taking each record's own
``x``/``y``/``hdg`` and walking its primitive: nothing re-anchors a record
onto its predecessor. So the point where one record ends has to be the point
where the next one starts, or the reference line has a step in it -- and the
ASAM rule ``road.geometry.contact_point``, which asks whether a road's
reference line begins at its neighbour's contact point, sees the same step at
the ends of the road.

``Line``/``Arc.from_spline_window`` anchor a record at the spline point and
tangent of ``s_start`` and give it ``length = s_end - s_start``. That is G1
at the start and wrong at the end twice over: a straight line that leaves
along the tangent of a spline which bends at all within the window does not
arrive at the spline point of ``s_end``, and the spline's arc length over the
window is not the primitive's own length either. Each record therefore ends
short of, or past, where the next one begins.

Measured on nishishinjuku with the classifier on (``arc_position_tol``
0.001 m, the retuned default): 529 of 9133 consecutive record pairs are more
than 1e-6 m apart, up to 1.766e-3 m, and 11 of the 12 ASAM errors are
``road.geometry.contact_point``. With the classifier off both are zero.
"""

import math
from typing import List, Tuple

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.conversion_config import ArcSpiralConfig
from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    Arc,
    GeometryBase,
    line_or_arc_from_spline_window,
)
from autoware_lanelet2_to_opendrive.opendrive.geometry_classifier import (
    ArcRun,
    LineRun,
    classify_spline,
)
from autoware_lanelet2_to_opendrive.spline import Splines

# The fit honours a position target to well under a micrometre, so anything
# above this is the record chain stepping, not arithmetic noise.
CONTINUITY_TOLERANCE_M = 1e-9


def _straightish_spline(
    length: float = 60.0, amp: float = 0.010, wavelength: float = 14.0, n: int = 121
) -> Splines:
    """Straight to a centimetre, but with curvature the classifier must ride.

    This is the #496 shape: a road digitised straight whose fitted spline
    oscillates. The classifier calls it a line, which is the right call --
    and exactly the case where a tangent-anchored line misses the far end.
    """
    x = np.linspace(0.0, length, n)
    y = amp * (1.0 - np.cos(2.0 * np.pi * x / wavelength))
    return Splines(np.column_stack([x, y, np.zeros(n)]))


def _curving_spline(radius: float = 60.0, arc: float = 0.9, n: int = 120) -> Splines:
    theta = np.linspace(0.0, arc, n)
    return Splines(
        np.column_stack([radius * np.cos(theta), radius * np.sin(theta), np.zeros(n)])
    )


def _record_end_xy(record) -> tuple:
    """Where the record itself ends, read the way a reader reads it."""
    if isinstance(record, Arc) and abs(record.curvature) > 1e-12:
        r = 1.0 / record.curvature
        dt = record.curvature * record.length
        return (
            record.x + r * (math.sin(record.hdg + dt) - math.sin(record.hdg)),
            record.y - r * (math.cos(record.hdg + dt) - math.cos(record.hdg)),
        )
    return (
        record.x + record.length * math.cos(record.hdg),
        record.y + record.length * math.sin(record.hdg),
    )


def _spline_xy(spline: Splines, s: float) -> tuple:
    point = spline.evaluate(s, derivative=0)
    return (float(point[0]), float(point[1]))


@pytest.mark.xfail(
    strict=True,
    reason="a plan-view record is anchored on its start and arrives wherever "
    "the primitive lands",
)
def test_straight_run_is_emitted_as_a_record_that_closes_the_window() -> None:
    """What the converter calls for a straight run has to close the window.

    ``Line`` itself cannot: a straight record carrying the window's arc
    length overshoots the far spline point by ``length - chord`` whenever the
    window bends at all, and no heading fixes that. The emitter therefore
    falls back to the arc that interpolates both ends -- while keeping the
    record's length equal to the window's arc length, so ``road@length``
    still equals the sum of its geometries.
    """
    # 12 m: short enough that the straight model stays inside
    # ``arc_position_tol``, so this is a window the classifier accepts
    # rather than one chosen to make the number large.
    spline = _straightish_spline()
    s_end = 12.0
    record = line_or_arc_from_spline_window(spline, s_start=0.0, s_end=s_end)

    assert record.length == pytest.approx(s_end, abs=1e-12), (
        "the record must carry the window's arc length, or the road's "
        "@length no longer equals the sum of its geometries"
    )
    gap = math.dist(_record_end_xy(record), _spline_xy(spline, s_end))
    assert gap < CONTINUITY_TOLERANCE_M, (
        f"the record ends {gap:.3e} m from the spline point at s={s_end}, "
        "so the next record -- which anchors on that spline point -- starts "
        "somewhere this one never reaches"
    )


@pytest.mark.xfail(
    strict=True,
    reason="a plan-view record is anchored on its start and arrives wherever "
    "the primitive lands",
)
def test_arc_record_ends_where_the_spline_window_ends() -> None:
    spline = _curving_spline()
    s_start, s_end = 5.0, 30.0
    curvature = 1.0 / 60.0
    arc = Arc.from_spline_window(
        spline, s_start=s_start, s_end=s_end, curvature=curvature
    )

    end = _record_end_xy(arc)
    want = _spline_xy(spline, s_end)
    gap = math.dist(end, want)
    assert (
        gap < CONTINUITY_TOLERANCE_M
    ), f"the arc record ends {gap:.3e} m from the spline point at s={s_end}"


@pytest.mark.xfail(
    strict=True,
    reason="a plan-view record is anchored on its start and arrives wherever "
    "the primitive lands",
)
def test_classified_records_form_one_unbroken_chain() -> None:
    """The property the two tests above exist to buy."""
    # A real curve, so the classifier has an arc to find; the straightish
    # shape above is classified as paramPoly3 throughout, which already
    # interpolates both ends and so proves nothing here.
    spline = _curving_spline(radius=60.0, arc=1.2, n=160)
    runs = classify_spline(
        spline,
        config=ArcSpiralConfig(enabled=True),
        constants=DEFAULT_CONFIG.arcspiral,
    )
    assert len(runs) >= 1

    # Keep each record next to the window it was built from: after the fix a
    # record's ``length`` is its own, not the window's arc length, so the
    # window bound cannot be read back off the record.
    built: List[Tuple[GeometryBase, float]] = []
    for run in runs:
        if isinstance(run, LineRun):
            built.append(
                (
                    line_or_arc_from_spline_window(spline, run.s_start, run.s_end),
                    run.s_end,
                )
            )
        elif isinstance(run, ArcRun):
            built.append(
                (
                    Arc.from_spline_window(
                        spline, run.s_start, run.s_end, run.curvature
                    ),
                    run.s_end,
                )
            )
    if not built:
        pytest.fail("the classifier produced no line or arc record to check")

    worst = 0.0
    where = ""
    for record, s_end in built:
        gap = math.dist(_record_end_xy(record), _spline_xy(spline, s_end))
        if gap > worst:
            worst = gap
            where = f"{type(record).__name__} covering s={record.s:.3f}..{s_end:.3f}"
    assert worst < CONTINUITY_TOLERANCE_M, (
        f"{where} ends {worst:.3e} m away from the spline point the next "
        "record anchors on, so the reference line steps there"
    )
