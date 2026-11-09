"""The emitted file must state the successions the source map states.

Geometry has ``symmetric_point_to_segment_max`` and conformance has the ASAM
bundle, but nothing in the repository measures what the map means for
driving: which lane you may reach from which. A conversion can place every
reference line to a fraction of a millimetre and still have dropped a turn,
and no gate we own would notice.

These tests pin the instrument and the figure. Two of them are hand-built
files that fix the two readings a route measurement gets wrong if it is
written naively:

* **Travel direction is the road's ``rule``.** Under left-hand traffic a
  positive-id (left) lane travels along +s and so reads its successor at the
  road's successor end; under right-hand traffic the negative-id lanes do.
  Reading the wrong end reports every transition in the file as missing.
* **A road with no backing lanelet is walked through, not counted.** The
  converter inserts a synthetic connector wherever the map states a
  succession directly. Counting it as a node reports each connector as a
  transition the map never stated *and* the succession it stands for as
  missing -- which on the reference map was a 650-of-974 figure that measured
  only the instrument.
"""

from pathlib import Path


from autoware_lanelet2_to_opendrive.route_fidelity import EmittedNetwork, measure

NISHISHINJUKU_MIN_SUCCESSIONS = 900
"""Below this the measurement did not run; the map states roughly 970."""

MAX_MISSING_SHARE = 0.02
"""The regression line, not a target.

Measured 1.55% here -- 15 of 969 successions on ``nishishinjuku`` as this
branch's base converts it. The same measurement reads 0.93% once the junction
and fork fixes land, so the bound is set above the worse of the two rather
than retightened every time the base moves.
"""


def _mapping_beside(xodr: Path) -> Path:
    """The conversion's mapping sidecar for ``xodr``.

    The converter names it after the output path it was given, and the
    nishishinjuku fixture converts to a staged name and renames the map
    afterwards -- so the sidecar can still carry the staged name. Both are
    accepted rather than pinning one, because which one appears is a property
    of the fixture, not of the thing under test.
    """
    direct = xodr.with_suffix(".mapping.json")
    if direct.is_file():
        return direct
    siblings = sorted(xodr.parent.glob("*.mapping.json"))
    assert (
        len(siblings) == 1
    ), f"expected one mapping sidecar beside {xodr}, found {siblings}"
    return siblings[0]


def _write(tmp_path: Path, name: str, body: str) -> Path:
    path = tmp_path / name
    path.write_text(body, encoding="utf-8")
    return path


def _road(road_id: int, rule: str, lanes: str, link: str = "") -> str:
    return f"""
  <road id="{road_id}" length="10.0" rule="{rule}">
    <link>{link}</link>
    <lanes>
      <laneSection s="0.0">{lanes}</laneSection>
    </lanes>
  </road>"""


def _lane(lane_id: int, link: str = "") -> str:
    side = "left" if lane_id > 0 else "right"
    return f"""
      <{side}>
        <lane id="{lane_id}" type="driving" level="false">
          <link>{link}</link>
        </lane>
      </{side}>"""


def test_a_left_hand_lane_leaves_by_the_successor_end(tmp_path: Path) -> None:
    """A positive-id lane under LHT travels along +s."""
    body = (
        '<?xml version="1.0"?><OpenDRIVE>'
        + _road(1, "LHT", _lane(1))
        + _road(2, "RHT", _lane(1))
        + "</OpenDRIVE>"
    )
    network = EmittedNetwork(_write(tmp_path, "rule.xodr", body))
    assert network.forward_end(1, 1) == "successor"
    assert network.forward_end(1, -1) == "predecessor"
    assert network.forward_end(2, 1) == "predecessor"
    assert network.forward_end(2, -1) == "successor"


def test_a_connector_with_no_backing_lanelet_is_walked_through(
    tmp_path: Path,
) -> None:
    """Road 1 -> connector 2 -> road 3 must be reported as 1 -> 3."""
    body = (
        '<?xml version="1.0"?><OpenDRIVE>'
        + _road(
            1,
            "LHT",
            _lane(1, '<successor id="1"/>'),
            '<successor elementType="road" elementId="2" contactPoint="start"/>',
        )
        + _road(
            2,
            "LHT",
            _lane(1, '<successor id="1"/>'),
            '<predecessor elementType="road" elementId="1" contactPoint="end"/>'
            '<successor elementType="road" elementId="3" contactPoint="start"/>',
        )
        + _road(
            3,
            "LHT",
            _lane(1),
            '<predecessor elementType="road" elementId="2" contactPoint="end"/>',
        )
        + "</OpenDRIVE>"
    )
    network = EmittedNetwork(_write(tmp_path, "connector.xodr", body))

    assert network.transitions(walk_through={2}) == {((1, 1), (3, 1))}
    # Counting the connector as a node is the mistake this pins against.
    assert ((1, 1), (2, 1)) in network.transitions(walk_through=set())


def test_shipped_conversion_preserves_the_stated_successions(
    nishishinjuku_xodr: Path,
) -> None:
    """The measured loss on the reference map stays where it is understood."""
    source = Path(__file__).parent / "data" / "nishishinjuku.osm"
    result = measure(source, nishishinjuku_xodr, _mapping_beside(nishishinjuku_xodr))

    assert result.stated >= NISHISHINJUKU_MIN_SUCCESSIONS, (
        f"only {result.stated} successions were read from the map;"
        " the measurement did not run"
    )
    assert result.missing_share < MAX_MISSING_SHARE, (
        f"{len(result.missing)} of {result.stated} successions"
        f" ({100.0 * result.missing_share:.2f}%) are absent from the file"
    )


# ---------------------------------------------------------------------------
# Dangling lane links
#
# A `<lane><link><successor id="N"/>` is read by looking N up in the lane
# section the road's successor end contacts. If no such lane exists there, a
# consumer dereferences nothing: CARLA exits 139. Nothing in the repository
# checks it, and the reader this module already builds knows both sides, so
# the check costs one pass over what is already parsed.
#
# The nishishinjuku output has none today, which is why this is a gate rather
# than a fix: it holds the zero while the junction and fork branches change
# how lane links are written.
# ---------------------------------------------------------------------------


def test_a_lane_link_naming_a_lane_that_does_not_exist_is_found(
    tmp_path: Path,
) -> None:
    """Road 1 lane 1 claims lane 2 of road 2, which has only lane 1."""
    body = (
        '<?xml version="1.0"?><OpenDRIVE>'
        + _road(
            1,
            "LHT",
            _lane(1, '<successor id="2"/>'),
            '<successor elementType="road" elementId="2" contactPoint="start"/>',
        )
        + _road(
            2,
            "LHT",
            _lane(1),
            '<predecessor elementType="road" elementId="1" contactPoint="end"/>',
        )
        + "</OpenDRIVE>"
    )
    network = EmittedNetwork(_write(tmp_path, "dangling.xodr", body))

    assert [
        (link.road, link.lane, link.end, link.target_road, link.target_lane)
        for link in network.dangling_lane_links()
    ] == [(1, 1, "successor", 2, 2)]


def test_a_lane_link_that_resolves_is_not_reported(tmp_path: Path) -> None:
    """The same file with the link pointing at the lane that exists."""
    body = (
        '<?xml version="1.0"?><OpenDRIVE>'
        + _road(
            1,
            "LHT",
            _lane(1, '<successor id="1"/>'),
            '<successor elementType="road" elementId="2" contactPoint="start"/>',
        )
        + _road(
            2,
            "LHT",
            _lane(1),
            '<predecessor elementType="road" elementId="1" contactPoint="end"/>',
        )
        + "</OpenDRIVE>"
    )
    network = EmittedNetwork(_write(tmp_path, "resolves.xodr", body))

    assert network.dangling_lane_links() == []


def test_a_lane_link_at_a_junction_end_is_not_reported(tmp_path: Path) -> None:
    """A junction-typed end resolves through the junction, not the lane link.

    Lane links are never cleared once written, so a road whose end became a
    junction still carries the id it had as a road-typed end. Reading it
    against the junction id would report every such end as dangling -- the
    false positive this pins against.
    """
    body = (
        '<?xml version="1.0"?><OpenDRIVE>'
        + _road(
            1,
            "LHT",
            _lane(1, '<successor id="7"/>'),
            '<successor elementType="junction" elementId="900"/>',
        )
        + _road(2, "LHT", _lane(1))
        + '<junction id="900"><connection id="0" incomingRoad="1"'
        ' connectingRoad="2" contactPoint="start">'
        '<laneLink from="1" to="1"/></connection></junction>' + "</OpenDRIVE>"
    )
    network = EmittedNetwork(_write(tmp_path, "junction.xodr", body))

    assert network.dangling_lane_links() == []


def test_shipped_conversion_has_no_dangling_lane_links(
    nishishinjuku_xodr: Path,
) -> None:
    """The gate: a lane link that names nothing is a segfault, not a loss."""
    network = EmittedNetwork(nishishinjuku_xodr)
    dangling = network.dangling_lane_links()

    assert dangling == [], (
        f"{len(dangling)} lane link(s) name a lane their target road does not"
        f" have; first few: {dangling[:5]}"
    )
