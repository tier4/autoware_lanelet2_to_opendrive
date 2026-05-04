"""Integration test for <lane><border> emission on nishishinjuku.osm (Issue #440)."""

from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    OriginSpec,
)
from autoware_lanelet2_to_opendrive.main import convert_lanelet2_to_opendrive


def test_nishishinjuku_emits_lane_border_for_asymmetric_lanelets(
    lanelet_map, capsys
):
    """Issue #440: at least one <lane><border> appears in the converted XODR,
    and the previously-emitted 'asymmetric lanelet' skip warnings disappear."""
    config = ConversionConfig(origin=OriginSpec(mgrs_code="54SUE"))
    opendrive_obj, *_ = convert_lanelet2_to_opendrive(lanelet_map, config)

    captured = capsys.readouterr()
    combined = captured.out + captured.err
    assert "asymmetric lanelet" not in combined.lower(), (
        "Asymmetric-lanelet skip warning must disappear after Issue #440"
    )

    xml_root = opendrive_obj.to_xml()
    border_count = len(xml_root.findall(".//lane/border"))
    assert border_count >= 1, (
        f"Expected at least 1 <lane><border> on nishishinjuku.osm "
        f"after Issue #440, got {border_count}"
    )
    # Upper bound is generous to avoid brittleness on minor map updates.
    assert border_count <= 500, (
        f"Border count outside expected range: {border_count}"
    )
