"""The ``target=default`` output must validate against the declared schema.

The asam-qc-opendrive bundle makes every semantic, geometry, performance and
smoothness checker depend on ``check_asam_xodr_xml_valid_schema`` through
``basic_preconditions.CHECKER_PRECONDITIONS``, and that precondition is
evaluated on the RAW issue list (``qc_baselib`` tests
``len(checker.issues) == 0``). One schema error therefore switches off 21 of
the bundle's 26 checkers, and no ignore-pattern can bring them back. These
tests pin the three constructs that used to break it, plus the end-to-end
invariant itself.
"""

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.opendrive.enums import LaneType
from autoware_lanelet2_to_opendrive.opendrive.header import Header
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.objects import CornerLocal

FIXTURE = (Path(__file__).parent / "data" / "walkway_mini.osm").resolve()


# ---------------------------------------------------------------------------
# Unit-level guards
# ---------------------------------------------------------------------------


def test_corner_local_emits_required_height() -> None:
    """``cornerLocal/@height`` is ``use="required"`` in every OpenDRIVE XSD."""
    elem = CornerLocal(u=1.0, v=-2.0, z=0.0).to_xml()
    assert elem.get("height") == "0.0"
    # The other three required attributes must stay present too.
    for attr in ("u", "v", "z"):
        assert elem.get(attr) is not None


def test_lane_does_not_emit_rule_attribute() -> None:
    """``rule`` is not an attribute of ``<lane>`` in any OpenDRIVE version.

    The RHT/LHT traffic rule belongs on ``<road rule>``; the value is still
    carried on the ``Lane`` object because it drives lane-id assignment, but
    it must not reach the XML.
    """
    lane = Lane(lane_id=-1, lane_type=LaneType.DRIVING, rule="LHT")
    assert lane.rule == "LHT"
    assert lane.to_xml().get("rule") is None


def test_header_declares_the_version_we_emit() -> None:
    """1.7 is the version whose XSD admits what the converter writes."""
    elem = Header().to_xml()
    assert elem.get("revMajor") == "1"
    assert elem.get("revMinor") == "7"


# ---------------------------------------------------------------------------
# End-to-end: the converted fixture must pass the bundle's schema checker
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def converted_default(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Convert the walkway fixture with the standard (non-CARLA) target."""
    out = tmp_path_factory.mktemp("schema") / "walkway_mini.xodr"
    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=example_mgrs_offset",
            "target=default",
            f"input_map_path={FIXTURE}",
            f"output_map_path={out}",
        ],
        check=True,
    )
    return out


def test_output_declares_1_7(converted_default: Path) -> None:
    header = ET.parse(str(converted_default)).getroot().find("header")
    assert header is not None
    assert (header.get("revMajor"), header.get("revMinor")) == ("1", "7")


def test_output_has_no_lane_rule_attribute(converted_default: Path) -> None:
    root = ET.parse(str(converted_default)).getroot()
    offenders = [lane for lane in root.iter("lane") if lane.get("rule") is not None]
    assert offenders == []


def test_output_schema_check_reports_no_issues(converted_default: Path) -> None:
    """The gate: zero issues, so the other 21 checkers are allowed to run."""
    from qc_baselib import Configuration, Result
    from qc_opendrive import constants
    from qc_opendrive.main import run_checks

    with tempfile.NamedTemporaryFile(suffix=".xqar", delete=False) as tmp:
        result_path = tmp.name
    config = Configuration()
    config.set_config_param("InputFile", str(converted_default))
    config.register_checker_bundle(constants.BUNDLE_NAME)
    config.set_checker_bundle_param(constants.BUNDLE_NAME, "resultFile", result_path)
    result = Result()
    result.register_checker_bundle(
        name=constants.BUNDLE_NAME,
        description="OpenDrive checker bundle",
        version=constants.BUNDLE_VERSION,
        summary="",
    )
    result.set_result_version(version=constants.BUNDLE_VERSION)
    run_checks(config, result)
    Path(result_path).unlink(missing_ok=True)

    schema = result.get_checker_result(
        constants.BUNDLE_NAME, "check_asam_xodr_xml_valid_schema"
    )
    messages = [
        loc.description
        for issue in schema.issues
        for loc in issue.locations
        if loc.description
    ]
    assert schema.issues == [], f"schema errors: {messages[:5]}"
