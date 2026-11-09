"""``lanelet_end_has_contact`` has to see a neighbour that holds its boundary
inverted.

Lanelet2 stores a boundary a lanelet traverses backwards as the *inverted*
view of a linestring, and ``laneletLayer.findUsages`` is keyed on the view the
lanelet registered: asked for the forward linestring -- which is the one the
``.osm`` loader puts in the ``lineStringLayer``, because that is the way as
digitised -- it returns nothing at all. So walking
point -> linestring -> lanelet misses exactly the longitudinal neighbours that
share only nodes, and the end reads as shared with nothing.

That answer decides the reference line's end tangent. A shared end takes the
end cut's perpendicular, which both roads derive from the same shared cut and
therefore agree on. An end shared with nothing takes the boundary's own
direction instead, on the grounds that there is nobody to agree with. Getting
it wrong puts the two roads' reference lines at an angle, and every lane
border, placed at ``ref(s) + t * normal(s)``, then lands in two places.

Measured on the customer map: 3,309 of 35,710 reference-lanelet ends (9.3%)
read as shared with nothing only because the inverted view was not asked for,
and the boundary direction they fall back to is up to 58.5 deg from the cut's
perpendicular. One traced pair of roads emitted reference lines 88.98 deg
apart at a contact whose two lanelets hold the same two node objects.
"""

from lanelet2.core import Lanelet, LaneletMap, LineString3d, Point3d, getId

from autoware_lanelet2_to_opendrive.util import lanelet_end_has_contact


def _chain(invert: bool):
    """Two lanelets sharing their whole end cut, reproducing the layer state a
    loaded map has: the forward linestrings live in the ``lineStringLayer``
    while each lanelet may hold the inverted view of them.

    ``invert`` reverses both lanelets, which is what a stretch digitised
    against the direction of travel looks like after ``lanelet2.io.load``.
    With it set, the shared cut is at ``leftBound[0]`` of the first lanelet.
    """
    left = [Point3d(getId(), x, 1.0, 0.0) for x in (0.0, 10.0, 20.0)]
    right = [Point3d(getId(), x, -1.0, 0.0) for x in (0.0, 10.0, 20.0)]
    a_left = LineString3d(getId(), left[:2])
    a_right = LineString3d(getId(), right[:2])
    b_left = LineString3d(getId(), left[1:])
    b_right = LineString3d(getId(), right[1:])

    def view(ls):
        return ls.invert() if invert else ls

    a = Lanelet(getId(), view(a_left), view(a_right))
    b = Lanelet(getId(), view(b_left), view(b_right))

    lanelet_map = LaneletMap()
    for ls in (a_left, a_right, b_left, b_right):
        lanelet_map.add(ls)
    lanelet_map.add(a)
    lanelet_map.add(b)
    # the end of ``a`` that the two lanelets share, and the one nothing touches
    return lanelet_map, a, invert, not invert


def test_a_forward_neighbour_is_found() -> None:
    """The control. With both lanelets stored forwards the walk already works,
    so a failure here would be the fixture rather than the behaviour."""
    lanelet_map, a, shared_at_start, lone_at_start = _chain(invert=False)
    assert lanelet_end_has_contact(
        lanelet_map, a, at_start=shared_at_start
    ), "the fixture does not express a shared end at all"
    assert not lanelet_end_has_contact(lanelet_map, a, at_start=lone_at_start), (
        "nothing shares that end, so it must read isolated -- the rule that "
        "falls back to the boundary's own tangent depends on this staying False"
    )


def test_an_inverted_neighbour_is_found() -> None:
    """A neighbour holding the inverted view shares the same node objects and
    the same cut, so the end is shared and has to read as shared."""
    lanelet_map, a, shared_at_start, lone_at_start = _chain(invert=True)

    index = 0 if shared_at_start else -1
    shared_point = a.leftBound[index]
    users = [
        other.id
        for ls in lanelet_map.lineStringLayer.findUsages(shared_point)
        for other in lanelet_map.laneletLayer.findUsages(ls)
    ]
    assert users == [], (
        "this test is only meaningful while the forward lookup finds nothing; "
        f"it found {users}, so the fixture no longer reproduces the defect"
    )

    assert lanelet_end_has_contact(lanelet_map, a, at_start=shared_at_start), (
        "the end is shared with the next lanelet -- they hold the same two "
        "node objects -- but the lookup asked only for the forward linestring, "
        "which neither lanelet registered; the reference line will now leave "
        "this end along its own boundary instead of the shared cut's "
        "perpendicular, and the neighbouring road will not agree"
    )
    assert not lanelet_end_has_contact(lanelet_map, a, at_start=lone_at_start), (
        "asking for the inverted view as well must not make a genuinely "
        "isolated end read as shared"
    )
