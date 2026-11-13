"""Tests for the two CARLA probes in ``scripts/``.

Neither test needs CARLA: the client is only imported inside ``main()``.
What is tested is everything that decides a count -- which roads are
measured, how an angle is classed, which mapping an answer is credited to --
with the consumer replaced by a stand-in that answers from a table.
"""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


# ``scripts/`` is deliberately not a package (see test_tdd.py); load by path.
def _load(name: str):
    spec = importlib.util.spec_from_file_location(
        name, Path(__file__).with_name(f"{name}.py")
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


yaw = _load("carla_connector_yaw")
judge = _load("carla_mapping_judge")

pytestmark = pytest.mark.xfail(
    strict=True, raises=NotImplementedError, reason="the probes are not written yet"
)


XODR = """<?xml version="1.0"?>
<OpenDRIVE>
  <road id="1" length="30.0" junction="-1">
    <lanes><laneSection s="0"><left><lane id="1" type="driving"/></left></laneSection></lanes>
  </road>
  <road id="2" length="0.01" junction="7">
    <link><predecessor elementType="road" elementId="1" contactPoint="end"/></link>
    <lanes><laneSection s="0"><left>
      <lane id="2" type="sidewalk"/><lane id="1" type="driving"/>
    </left></laneSection></lanes>
  </road>
  <road id="3" length="0.02" junction="7">
    <link><predecessor elementType="junction" elementId="7"/></link>
    <lanes><laneSection s="0"><right><lane id="-1" type="driving"/></right></laneSection></lanes>
  </road>
  <road id="4" length="0.03" junction="-1">
    <lanes><laneSection s="0"><left><lane id="1" type="sidewalk"/></left></laneSection></lanes>
  </road>
</OpenDRIVE>
"""


class FakeCarlaMap:
    """Answers ``get_waypoint_xodr`` from ``{(road, lane): yaw}``."""

    def __init__(self, yaws: dict) -> None:
        self.yaws = yaws
        self.asked: list = []

    def get_waypoint_xodr(self, road: int, lane: int, s: float):
        self.asked.append((road, lane, s))
        if (road, lane) not in self.yaws:
            return None
        rotation = SimpleNamespace(yaw=self.yaws[(road, lane)])
        return SimpleNamespace(transform=SimpleNamespace(rotation=rotation))


@pytest.fixture
def xodr(tmp_path: Path) -> str:
    path = tmp_path / "map.xodr"
    path.write_text(XODR, encoding="utf-8")
    return str(path)


class TestConnectorYaw:
    def test_only_short_roads_with_a_driving_lane_are_read(self, xodr: str) -> None:
        """Road 1 is long; road 4 is short but carries no driving lane."""
        short, lengths = yaw.read_short_roads(xodr, 0.5)
        assert short == [(2, 0.01, [1], (1, "end")), (3, 0.02, [-1], None)]
        assert lengths == {1: 30.0, 2: 0.01, 3: 0.02, 4: 0.03}

    @pytest.mark.parametrize(
        ("a", "b", "expected"),
        [
            (0.0, 0.0, 0.0),
            (170.0, -170.0, 20.0),
            (-90.0, 90.0, 180.0),
            (350.0, 10.0, 20.0),
        ],
    )
    def test_angle_between_wraps(self, a: float, b: float, expected: float) -> None:
        assert yaw.angle_between(a, b) == pytest.approx(expected)

    @pytest.mark.parametrize(
        ("delta", "expected"),
        [
            (0.0, "agrees within 5 deg"),
            (5.0, "agrees within 5 deg"),
            (5.1, "off by 5 to 80 deg"),
            (80.1, "crosswise (80 to 150 deg)"),
            (150.0, "crosswise (80 to 150 deg)"),
            (179.9, "reversed (over 150 deg)"),
        ],
    )
    def test_classes(self, delta: float, expected: str) -> None:
        assert yaw.classify(delta) == expected

    def test_a_reversed_connector_is_counted_as_reversed(self, xodr: str) -> None:
        """Road 2 faces 180 degrees away from the road feeding it.

        Road 3 has no road predecessor: it is reported as not comparable, not
        silently dropped, so the denominator stays visible.
        """
        short, lengths = yaw.read_short_roads(xodr, 0.5)
        carla_map = FakeCarlaMap({(1, 1): 10.0, (2, 1): -170.0, (3, -1): 0.0})
        verdict, deltas = yaw.measure(carla_map, short, lengths)
        assert dict(verdict) == {
            "reversed (over 150 deg)": 1,
            "no road predecessor to compare against": 1,
        }
        assert deltas == [pytest.approx(180.0)]
        # the feeder is read 5 cm inside its own end, the connector at its middle
        assert (1, 1, pytest.approx(29.95)) in carla_map.asked
        assert (2, 1, pytest.approx(0.005)) in carla_map.asked

    def test_report_states_the_denominator(self, xodr: str) -> None:
        short, lengths = yaw.read_short_roads(xodr, 0.5)
        carla_map = FakeCarlaMap({(1, 1): 10.0, (2, 1): 12.0, (3, -1): 0.0})
        verdict, deltas = yaw.measure(carla_map, short, lengths)
        text = yaw.report("abc1234", 0.5, len(short), verdict, deltas)
        assert text.splitlines()[0] == "===== abc1234 ====="
        assert "roads under 0.5 m carrying a driving lane: 2" in text
        assert "driving lanes on them: 2; compared with a feeding road: 1" in text


class TestMappingJudge:
    @pytest.mark.parametrize(
        ("answer", "conversion", "cross_check", "expected"),
        [
            ((5, 1), (5, 1), (5, 1), "carla = both"),
            ((5, 1), (5, 1), (5, 2), "carla = conversion mapping"),
            ((5, 1), (5, 1), None, "carla = conversion mapping"),
            ((5, 2), (5, 1), (5, 2), "carla = geometric cross-check"),
            ((5, 3), (5, 1), (5, 2), "carla = conversion's road, a third lane"),
            ((9, 1), (5, 1), (5, 2), "carla = another road"),
            (None, (5, 1), (5, 2), "carla has no waypoint there"),
        ],
    )
    def test_verdict(self, answer, conversion, cross_check, expected: str) -> None:
        assert judge.verdict(answer, conversion, cross_check) == expected

    @pytest.mark.parametrize(
        ("conversion", "cross_check", "expected"),
        [
            ((5, 1), (5, 1), None),
            ((5, 1), (5, 2), "same road, other lane"),
            ((5, 1), (6, 1), "other road"),
            ((5, 1), None, "the cross-check has no answer"),
        ],
    )
    def test_disagreement_kind(self, conversion, cross_check, expected) -> None:
        assert judge.disagreement_kind(conversion, cross_check) == expected

    def test_two_sided_roads(self) -> None:
        conversion = {10: (5, 1), 11: (5, -1), 12: (6, 1), 13: (6, 2)}
        assert judge.two_sided_roads(conversion) == {5}

    def test_judge_tallies_each_lanelet_once(self) -> None:
        conversion = {10: (5, 1), 11: (5, 2), 12: (6, 1)}
        cross_check = {10: (5, 2), 11: (5, 2)}
        answers = {10: ((5, 1), 0.1), 11: ((5, 2), 0.2), 12: (None, None)}
        tally, distances = judge.judge(
            [10, 11, 12], conversion, cross_check, answers.__getitem__
        )
        assert dict(tally) == {
            "carla = conversion mapping": 1,
            "carla = both": 1,
            "carla has no waypoint there": 1,
        }
        assert distances == [0.1, 0.2]

    def test_block_carries_size_and_shares(self) -> None:
        tally, distances = judge.judge(
            [10, 11],
            {10: (5, 1), 11: (5, 2)},
            {10: (5, 2), 11: (5, 2)},
            {10: ((5, 1), 0.1), 11: ((5, 2), 0.3)}.__getitem__,
        )
        text = judge.format_block("DISAGREE: x", tally, distances)
        assert text.splitlines()[0].startswith("DISAGREE: x  (n=2; ")
        assert "  50.0%  carla = conversion mapping" in text
