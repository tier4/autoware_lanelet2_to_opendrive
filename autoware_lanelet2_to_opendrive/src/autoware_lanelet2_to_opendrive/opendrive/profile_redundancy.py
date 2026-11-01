"""Drop a profile record that restates its predecessor's polynomial.

A lane ``<width>`` and a road ``<elevation>`` are cubics in ``s`` measured
from their own offset, and each runs until the next record starts. So two
consecutive records that evaluate to the same value at every ``s`` are the
same curve written twice, and the second one carries no information: deleting
it leaves the covered range and every value unchanged.

That is also exactly what the ASAM rule
``performance.avoid_redundant_info`` reports, and on nishishinjuku it is the
whole warning population -- 341 of 342 issues, 108 in ``<width>`` and 233 in
``<elevation>``.

**Only a redundant record may be dropped.** A lane whose ``<width>`` records
do not cover its whole ``s`` makes consumers abort, and dropping a record
whose polynomial differs would change the surface. This module therefore
compares the two cubics rather than their coefficients: ``a=3.0, b=0.1`` at
offset 0 and ``a=4.0, b=0.1`` at offset 10 are the same line, which a
coefficient comparison would keep, while ``a=3.0`` and ``a=3.0, c=1e-4`` are
not, which a coefficient comparison with a loose tolerance might drop.
"""

from __future__ import annotations

from typing import List, Protocol, Sequence, TypeVar

from ..config import DEFAULT_CONFIG


class _Poly3Record(Protocol):
    """A cubic in ``s`` with its own offset; the offset's name varies."""

    a: float
    b: float
    c: float
    d: float


RecordT = TypeVar("RecordT", bound=_Poly3Record)


def states_the_same_polynomial(
    first_offset: float,
    first: _Poly3Record,
    second_offset: float,
    second: _Poly3Record,
    tolerance: float | None = None,
) -> bool:
    """Do the two records describe one curve?

    Expand ``f1(s) - f2(s)`` with ``f_i(s) = a_i + b_i*(s - o_i) + ...`` and
    require every coefficient of the difference to vanish. This mirrors
    ``qc_opendrive.base.utils.are_same_equations`` term for term, so a record
    this keeps is a record the checker does not report.
    """
    if tolerance is None:
        tolerance = DEFAULT_CONFIG.opendrive.redundant_record_tolerance

    o1, o2 = first_offset, second_offset
    constant = (
        first.a
        - second.a
        - first.b * o1
        + second.b * o2
        + first.c * o1**2
        - second.c * o2**2
        - first.d * o1**3
        + second.d * o2**3
    )
    linear = (
        first.b
        - second.b
        - 2.0 * first.c * o1
        + 2.0 * second.c * o2
        + 3.0 * first.d * o1**2
        - 3.0 * second.d * o2**2
    )
    quadratic = first.c - second.c - 3.0 * first.d * o1 + 3.0 * second.d * o2
    cubic = first.d - second.d
    return all(abs(term) < tolerance for term in (constant, linear, quadratic, cubic))


def without_redundant_records(
    records: Sequence[RecordT],
    offset_attr: str,
    tolerance: float | None = None,
) -> List[RecordT]:
    """Return ``records`` with every restatement of its predecessor removed.

    Comparison is against the record that will actually precede it in the
    output -- the last one kept -- not against the one it followed in the
    input. A run of five records stating one curve therefore collapses to the
    first, rather than to every other one.

    Args:
        records: Profile records in increasing offset order.
        offset_attr: Name of the offset attribute (``s_offset`` for a lane
            ``<width>``, ``s`` for a road ``<elevation>``).
        tolerance: Override for the coefficient tolerance; defaults to
            ``DEFAULT_CONFIG.opendrive.redundant_record_tolerance``.

    Returns:
        A new list holding the records to write. The first record is always
        kept: something has to cover ``s = 0``.
    """
    kept: List[RecordT] = []
    for record in records:
        if kept:
            previous = kept[-1]
            if states_the_same_polynomial(
                float(getattr(previous, offset_attr)),
                previous,
                float(getattr(record, offset_attr)),
                record,
                tolerance,
            ):
                continue
        kept.append(record)
    return kept
