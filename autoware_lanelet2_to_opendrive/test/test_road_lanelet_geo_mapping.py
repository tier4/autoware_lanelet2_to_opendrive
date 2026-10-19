"""Tests for road_lanelet_geo_mapping: MappingMismatchError, validate, save."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    Arc,
    Line,
    ParamPoly3,
    PlanView,
)
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    _MATCH_THRESHOLD,
    GeoRoadLaneletMapping,
    MappingMismatchError,
    ReferenceMatchDistance,
    _bbox,
    _bboxes_overlap,
    _build_boundary_grid,
    _CANDIDATE_BBOX_MARGIN,
    _grid_candidate_positions,
    _RoadCandidates,
    _REPORT_METRIC_NAME,
    _resolve_conflicts,
    _sample_reference_line_from_road,
    _symmetric_max_polyline_distance,
    _symmetric_mean_distance,
    _symmetric_mean_polyline_distance,
    build_mapping,
    parse_roads_from_xodr,
    save_mapping_json,
    summarise_reference_match_accuracy,
    validate_mapping_consistency,
)


# ---------------------------------------------------------------------------
# validate_mapping_consistency
# ---------------------------------------------------------------------------


class TestValidateMappingConsistency:
    def test_matching_mappings_pass(self) -> None:
        conversion = {10: (1, -1), 20: (2, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -1)},
        )
        # Should not raise
        validate_mapping_consistency(conversion, geo)

    def test_empty_mappings_pass(self) -> None:
        geo = GeoRoadLaneletMapping(xodr_sha256="a", osm_sha256="b")
        validate_mapping_consistency({}, geo)

    def test_value_mismatch_raises(self) -> None:
        conversion = {10: (1, -1), 20: (2, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -2)},
        )
        with pytest.raises(MappingMismatchError, match="1 entries differ"):
            validate_mapping_consistency(conversion, geo)

    def test_missing_in_geo_raises(self) -> None:
        conversion = {10: (1, -1), 20: (2, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -1)},
        )
        with pytest.raises(MappingMismatchError, match="1 entries differ"):
            validate_mapping_consistency(conversion, geo)

    def test_missing_in_conversion_raises(self) -> None:
        conversion = {10: (1, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -1)},
        )
        with pytest.raises(MappingMismatchError, match="1 entries differ"):
            validate_mapping_consistency(conversion, geo)

    def test_multiple_mismatches(self) -> None:
        conversion = {10: (1, -1), 20: (2, -1), 30: (3, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -2), 20: (2, -2), 30: (3, -2)},
        )
        with pytest.raises(MappingMismatchError, match="3 entries differ"):
            validate_mapping_consistency(conversion, geo)

    def test_strict_returns_true_on_match(self) -> None:
        conversion = {10: (1, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -1)},
        )
        assert validate_mapping_consistency(conversion, geo) is True

    def test_non_strict_returns_false_instead_of_raising(self) -> None:
        conversion = {10: (1, -1), 20: (2, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -2)},
        )
        assert validate_mapping_consistency(conversion, geo, strict=False) is False

    def test_non_strict_logs_the_mismatch_detail(self, caplog) -> None:
        conversion = {10: (1, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -2)},
        )
        with caplog.at_level("WARNING"):
            validate_mapping_consistency(conversion, geo, strict=False)
        assert "1 entries differ" in caplog.text
        assert "lanelet 10" in caplog.text


# ---------------------------------------------------------------------------
# save_mapping_json
# ---------------------------------------------------------------------------


class TestSaveMappingJson:
    def test_creates_json_file(self, tmp_path: Path) -> None:
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        mapping = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -1)},
        )
        result_path = save_mapping_json(mapping, xodr_path)

        expected_path = tmp_path / "test.mapping.json"
        assert result_path == expected_path
        assert expected_path.exists()

    def test_json_content_valid(self, tmp_path: Path) -> None:
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        mapping = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -1)},
        )
        result_path = save_mapping_json(mapping, xodr_path)

        data = json.loads(result_path.read_text(encoding="utf-8"))
        assert "version" not in data
        assert data["xodr_sha256"] == "abc"
        assert data["osm_sha256"] == "def"
        assert "10" in data["lanelet_to_road_and_lane"]
        assert data["lanelet_to_road_and_lane"]["10"] == [1, -1]

    def test_round_trip_from_saved_json(self, tmp_path: Path) -> None:
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        original = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -1)},
        )
        result_path = save_mapping_json(original, xodr_path)

        data = json.loads(result_path.read_text(encoding="utf-8"))
        restored = GeoRoadLaneletMapping.from_dict(data)
        assert restored.lanelet_to_road_and_lane == original.lanelet_to_road_and_lane
        assert restored.xodr_sha256 == original.xodr_sha256
        assert restored.osm_sha256 == original.osm_sha256

    def test_round_trip_preserves_skipped_synthetic_roads(self, tmp_path: Path) -> None:
        """#493: synthetic divergence connectors are recorded in the mapping
        JSON so consumers can tell them apart from real mapping failures."""
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        original = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1)},
            skipped_synthetic_roads=[274, 290, 303],
        )
        result_path = save_mapping_json(original, xodr_path)

        data = json.loads(result_path.read_text(encoding="utf-8"))
        assert data["skipped_synthetic_roads"] == [274, 290, 303]
        restored = GeoRoadLaneletMapping.from_dict(data)
        assert restored.skipped_synthetic_roads == [274, 290, 303]

    def test_round_trip_preserves_reference_match_distances(
        self, tmp_path: Path
    ) -> None:
        """Positional error against the Lanelet2 input survives the JSON."""
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        records = {
            10: ReferenceMatchDistance(
                max_m=0.1234, mean_m=0.05, boundary_length_m=40.0
            ),
            11: ReferenceMatchDistance(max_m=2.5, mean_m=0.4, boundary_length_m=17.5),
        }
        original = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1)},
            reference_match_distances=records,
        )
        result_path = save_mapping_json(original, xodr_path)

        data = json.loads(result_path.read_text(encoding="utf-8"))
        assert data["reference_match_distances"]["10"] == {
            "max_m": 0.1234,
            "mean_m": 0.05,
            "boundary_length_m": 40.0,
        }
        restored = GeoRoadLaneletMapping.from_dict(data)
        assert restored.reference_match_distances == records

    def test_round_trip_preserves_rescued_reference_lanelets(
        self, tmp_path: Path
    ) -> None:
        """Rescued matches are flagged so a report cannot claim they are
        within the matching tolerance — the rescue pass accepts more."""
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        original = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1)},
            reference_match_distances={
                10: ReferenceMatchDistance(max_m=0.5, mean_m=0.2),
                11: ReferenceMatchDistance(max_m=4.6, mean_m=4.6),
            },
            rescued_reference_lanelets=[11],
        )
        result_path = save_mapping_json(original, xodr_path)

        data = json.loads(result_path.read_text(encoding="utf-8"))
        assert data["rescued_reference_lanelets"] == [11]
        restored = GeoRoadLaneletMapping.from_dict(data)
        assert restored.rescued_reference_lanelets == [11]

    def test_absent_reference_match_distances_stay_absent(self, tmp_path: Path) -> None:
        """Mappings from before this field existed round-trip unchanged."""
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        original = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1)},
        )
        result_path = save_mapping_json(original, xodr_path)

        data = json.loads(result_path.read_text(encoding="utf-8"))
        assert "reference_match_distances" not in data
        assert GeoRoadLaneletMapping.from_dict(data).reference_match_distances is None

    def test_legacy_float_reference_match_distances_still_load(self) -> None:
        """A mapping written before the record existed stored a bare float.

        That float was the mean; the maximum cannot be recovered from it, so
        it stands in for both rather than the whole record being dropped.
        """
        restored = GeoRoadLaneletMapping.from_dict(
            {
                "xodr_sha256": "abc",
                "osm_sha256": "def",
                "lanelet_to_road_and_lane": {"10": [1, -1]},
                "reference_match_distances": {"10": 0.25, "11": 1.5},
            }
        )
        assert restored.reference_match_distances == {
            10: ReferenceMatchDistance(max_m=0.25, mean_m=0.25, boundary_length_m=None),
            11: ReferenceMatchDistance(max_m=1.5, mean_m=1.5, boundary_length_m=None),
        }

        # And it still summarises — minus the density, which needs the
        # boundary lengths the old format never recorded.
        summary = summarise_reference_match_accuracy(restored)
        assert summary["count"] == 2
        assert summary["max_m"] == pytest.approx(1.5)
        assert summary["matched_boundary_length_m"] is None
        assert all(exc["per_km"] is None for exc in summary["exceedances"])


# ---------------------------------------------------------------------------
# summarise_reference_match_accuracy
# ---------------------------------------------------------------------------


#: Configured exceedance levels, ordered least to most severe.
_EXCEEDANCE_LEVELS = DEFAULT_CONFIG.geo_mapping.exceedance_thresholds_m


def _mapping_with_distances(
    distances: dict[int, float],
    rescued: list[int] | None = None,
    means: dict[int, float] | None = None,
    boundary_lengths: dict[int, float] | None = None,
) -> GeoRoadLaneletMapping:
    """Mapping whose per-lanelet records carry ``distances`` as the maxima."""
    records = {
        lid: ReferenceMatchDistance(
            max_m=d,
            mean_m=(means or {}).get(lid, d),
            boundary_length_m=(boundary_lengths or {}).get(lid),
        )
        for lid, d in distances.items()
    }
    return GeoRoadLaneletMapping(
        xodr_sha256="abc",
        osm_sha256="def",
        lanelet_to_road_and_lane={lid: (lid, -1) for lid in distances},
        reference_match_distances=records or None,
        rescued_reference_lanelets=rescued,
    )


class TestSummariseReferenceMatchAccuracy:
    def test_no_distances_returns_empty(self) -> None:
        assert summarise_reference_match_accuracy(_mapping_with_distances({})) == {}

    def test_reports_distribution(self) -> None:
        summary = summarise_reference_match_accuracy(
            _mapping_with_distances({1: 0.1, 2: 0.2, 3: 0.3})
        )
        assert summary["count"] == 3
        assert summary["median_m"] == pytest.approx(0.2)
        assert summary["max_m"] == pytest.approx(0.3)
        assert summary["match_threshold_m"] == _MATCH_THRESHOLD

    def test_warn_threshold_defaults_to_half_the_matching_tolerance(self) -> None:
        summary = summarise_reference_match_accuracy(_mapping_with_distances({1: 0.1}))
        assert summary["warn_threshold_m"] == pytest.approx(_MATCH_THRESHOLD / 2.0)

    def test_lists_worst_offenders_sorted_by_distance(self) -> None:
        """A match just inside the tolerance passes matching but is not accurate."""
        near_limit = _MATCH_THRESHOLD - 0.1
        summary = summarise_reference_match_accuracy(
            _mapping_with_distances({1: 0.05, 2: near_limit, 3: _MATCH_THRESHOLD / 2.0})
        )
        assert [w["lanelet_id"] for w in summary["worst"]] == [2, 3]
        assert summary["worst"][0]["distance_m"] == pytest.approx(near_limit)

    def test_accurate_map_reports_no_offenders(self) -> None:
        summary = summarise_reference_match_accuracy(
            _mapping_with_distances({1: 0.01, 2: 0.02, 3: 0.03})
        )
        assert summary["worst"] == []

    def test_explicit_warn_threshold_is_honoured(self) -> None:
        summary = summarise_reference_match_accuracy(
            _mapping_with_distances({1: 0.5, 2: 1.5}), warn_threshold=1.0
        )
        assert [w["lanelet_id"] for w in summary["worst"]] == [2]
        assert summary["warn_threshold_m"] == 1.0

    def test_thresholds_come_from_the_geo_mapping_config(self) -> None:
        summary = summarise_reference_match_accuracy(_mapping_with_distances({1: 0.1}))
        geo = DEFAULT_CONFIG.geo_mapping
        assert summary["warn_threshold_m"] == pytest.approx(
            geo.match_threshold * geo.warn_threshold_factor
        )
        assert summary["rescue_threshold_m"] == pytest.approx(
            geo.match_threshold * geo.rescue_threshold_factor
        )

    def test_metric_name_is_reported(self) -> None:
        """All values share one metric, so the summary can name it."""
        summary = summarise_reference_match_accuracy(_mapping_with_distances({1: 0.1}))
        assert summary["metric"] == _REPORT_METRIC_NAME

    def test_rescued_matches_above_the_tolerance_are_counted(self) -> None:
        """A rescued road may sit beyond the matching tolerance; saying every
        match is "within tolerance" would be wrong for it."""
        rescue_dist = _MATCH_THRESHOLD + 1.0
        summary = summarise_reference_match_accuracy(
            _mapping_with_distances({1: 0.1, 2: rescue_dist}, rescued=[2])
        )
        assert summary["rescued_count"] == 1
        assert summary["above_match_threshold"] == 1
        assert summary["max_m"] == pytest.approx(rescue_dist)
        assert summary["worst"][0] == {
            "lanelet_id": 2,
            "distance_m": pytest.approx(rescue_dist),
            "mean_distance_m": pytest.approx(rescue_dist),
            "rescued": True,
        }

    def test_non_rescued_matches_are_flagged_as_such(self) -> None:
        summary = summarise_reference_match_accuracy(
            _mapping_with_distances({1: _MATCH_THRESHOLD - 0.1})
        )
        assert summary["rescued_count"] == 0
        assert summary["above_match_threshold"] == 0
        assert summary["worst"][0]["rescued"] is False

    def test_absolute_max_names_the_lanelet_it_belongs_to(self) -> None:
        """The headline max is actionable only if it says where to look."""
        summary = summarise_reference_match_accuracy(
            _mapping_with_distances({1: 0.1, 2: 1.9, 3: 0.4})
        )
        assert summary["max_m"] == pytest.approx(1.9)
        assert summary["worst_lanelet_id"] == 2

    def test_secondary_statistics_are_labelled_scale_dependent(self) -> None:
        """Median and p95 stay, but flagged: they move when easy roads are
        added, so they cannot be compared between maps."""
        summary = summarise_reference_match_accuracy(
            _mapping_with_distances({1: 0.1, 2: 0.2, 3: 0.3})
        )
        assert "median_m" in summary["scale_dependent_keys"]
        assert "p95_m" in summary["scale_dependent_keys"]

    def test_median_is_scale_dependent_but_the_count_is_not(self) -> None:
        """Adding well-converted roads must not make the map look better.

        The same three bad roads are reported identically once ten good ones
        are added; only the median improves, which is exactly why it is not
        the headline.
        """
        bad = {1: 0.6, 2: 1.4, 3: 2.0}
        small = summarise_reference_match_accuracy(_mapping_with_distances(bad))
        padded = summarise_reference_match_accuracy(
            _mapping_with_distances({**bad, **{100 + i: 0.01 for i in range(10)}})
        )

        counts_small = [exc["count"] for exc in small["exceedances"]]
        counts_padded = [exc["count"] for exc in padded["exceedances"]]
        assert counts_small == counts_padded
        assert padded["max_m"] == pytest.approx(small["max_m"])
        assert padded["median_m"] < small["median_m"]

    def test_counts_roads_above_each_configured_threshold(self) -> None:
        inaccuracy, error = _EXCEEDANCE_LEVELS
        summary = summarise_reference_match_accuracy(
            _mapping_with_distances(
                {
                    1: 0.05,  # below both
                    2: inaccuracy[1] + 0.1,  # modelling inaccuracy only
                    3: error[1] + 0.1,  # both
                    4: error[1] + 2.0,  # both
                }
            )
        )
        by_label = {exc["label"]: exc for exc in summary["exceedances"]}
        assert by_label[inaccuracy[0]]["threshold_m"] == pytest.approx(inaccuracy[1])
        assert by_label[inaccuracy[0]]["count"] == 3
        assert by_label[inaccuracy[0]]["fraction"] == pytest.approx(0.75)
        assert by_label[error[0]]["count"] == 2
        assert by_label[error[0]]["fraction"] == pytest.approx(0.5)

    def test_thresholds_come_from_config_not_hardcoded(self) -> None:
        summary = summarise_reference_match_accuracy(_mapping_with_distances({1: 0.1}))
        assert [
            (exc["label"], exc["threshold_m"]) for exc in summary["exceedances"]
        ] == [(label, pytest.approx(value)) for label, value in _EXCEEDANCE_LEVELS]

    def test_exceedance_density_is_per_kilometre_of_matched_boundary(self) -> None:
        """Two roads over the error threshold in 500 m is 4 per km."""
        error_label, error_threshold = _EXCEEDANCE_LEVELS[1]
        summary = summarise_reference_match_accuracy(
            _mapping_with_distances(
                {1: 0.05, 2: error_threshold + 0.5, 3: error_threshold + 0.5},
                boundary_lengths={1: 200.0, 2: 200.0, 3: 100.0},
            )
        )
        assert summary["matched_boundary_length_m"] == pytest.approx(500.0)
        by_label = {exc["label"]: exc for exc in summary["exceedances"]}
        assert by_label[error_label]["per_km"] == pytest.approx(4.0)

    def test_density_is_independent_of_map_size(self) -> None:
        """A map twice the size with twice the problems scores the same."""
        _, threshold = _EXCEEDANCE_LEVELS[1]
        bad = threshold + 0.5
        small = summarise_reference_match_accuracy(
            _mapping_with_distances(
                {1: 0.05, 2: bad}, boundary_lengths={1: 500.0, 2: 500.0}
            )
        )
        large = summarise_reference_match_accuracy(
            _mapping_with_distances(
                {1: 0.05, 2: bad, 3: 0.05, 4: bad},
                boundary_lengths={1: 500.0, 2: 500.0, 3: 500.0, 4: 500.0},
            )
        )
        assert large["exceedances"][1]["per_km"] == pytest.approx(
            small["exceedances"][1]["per_km"]
        )
        assert large["exceedances"][1]["count"] == 2 * small["exceedances"][1]["count"]


class TestSummaryHeadlineIsScaleIndependent:
    """The headline figure must not improve when well-converted roads appear.

    ``median_m`` and ``p95_m`` describe the mix of roads in one particular
    map. Convert a wider area that happens to contain more easy roads and both
    fall, with nothing having been fixed, so two maps cannot be compared by
    them and a regression can hide behind growth. The headline therefore has
    to be a count of roads past each severity level, which does not move when
    accurate roads are added.
    """

    def test_adding_accurate_roads_leaves_the_headline_alone(self) -> None:
        offenders = {1: 0.2, 2: 1.4}
        small = summarise_reference_match_accuracy(_mapping_with_distances(offenders))
        large = summarise_reference_match_accuracy(
            _mapping_with_distances({**offenders, **{i: 0.01 for i in range(10, 60)}})
        )

        assert "exceedances" in small, (
            "the summary has no count-based headline: median_m and p95_m alone "
            "cannot be compared between maps of different size"
        )
        assert [e["count"] for e in small["exceedances"]] == [
            e["count"] for e in large["exceedances"]
        ]
        # The statistics this guards against do move, which is the point.
        assert large["median_m"] < small["median_m"]


# ---------------------------------------------------------------------------
# Reporting distance metric (point-to-segment vs point-to-vertex)
# ---------------------------------------------------------------------------


def _polyline(y: float, x_values: list[float]) -> np.ndarray:
    """Horizontal polyline at height ``y`` through ``x_values``."""
    return np.array([[float(x), float(y)] for x in x_values])


class TestReportDistanceMetric:
    """The reported accuracy must not charge boundary vertex spacing as error.

    A Lanelet2 boundary is a handful of vertices metres apart; the road
    reference line is sampled every 0.5 m.  Measured against the boundary's
    *vertices*, a reference line lying exactly on the boundary still reports
    roughly a quarter of the vertex spacing.
    """

    #: Boundary vertices 5 m apart, reference line sampled every 0.5 m.
    SPARSE_X = [float(x) for x in range(0, 105, 5)]
    DENSE_X = [0.5 * i for i in range(201)]

    def test_perfect_fit_over_sparse_vertices_reports_zero(self) -> None:
        boundary = _polyline(0.0, self.SPARSE_X)
        ref_line = _polyline(0.0, self.DENSE_X)
        assert _symmetric_mean_polyline_distance(ref_line, boundary) == pytest.approx(
            0.0, abs=1e-9
        )

    def test_matching_metric_is_left_unchanged(self) -> None:
        """The vertex-based metric still reports ~1.25 m for the same input.

        It is deliberately untouched: it decides matches, and this fix is a
        reporting fix only.
        """
        boundary = _polyline(0.0, self.SPARSE_X)
        ref_line = _polyline(0.0, self.DENSE_X)
        assert _symmetric_mean_distance(ref_line, boundary) == pytest.approx(
            1.24, abs=0.01
        )

    def test_constant_offset_is_reported_exactly(self) -> None:
        boundary = _polyline(0.0, self.SPARSE_X)
        ref_line = _polyline(0.2, self.DENSE_X)
        assert _symmetric_mean_polyline_distance(ref_line, boundary) == pytest.approx(
            0.2
        )

    def test_two_point_boundary_is_handled(self) -> None:
        boundary = _polyline(0.0, [0.0, 100.0])
        ref_line = _polyline(0.0, self.DENSE_X)
        assert _symmetric_mean_polyline_distance(ref_line, boundary) == pytest.approx(
            0.0, abs=1e-9
        )

    def test_duplicate_vertices_do_not_divide_by_zero(self) -> None:
        boundary = np.array([[0.0, 0.0], [0.0, 0.0], [100.0, 0.0]])
        ref_line = _polyline(0.0, self.DENSE_X)
        assert _symmetric_mean_polyline_distance(ref_line, boundary) == pytest.approx(
            0.0, abs=1e-9
        )

    def test_max_form_agrees_with_the_mean_on_a_constant_offset(self) -> None:
        """With no variation along the road the two forms coincide."""
        boundary = _polyline(0.0, self.SPARSE_X)
        ref_line = _polyline(0.2, self.DENSE_X)
        assert _symmetric_max_polyline_distance(ref_line, boundary) == pytest.approx(
            0.2
        )

    def test_max_form_handles_a_two_point_boundary(self) -> None:
        boundary = _polyline(0.0, [0.0, 100.0])
        ref_line = _polyline(0.0, self.DENSE_X)
        assert _symmetric_max_polyline_distance(ref_line, boundary) == pytest.approx(
            0.0, abs=1e-9
        )


# ---------------------------------------------------------------------------
# build_mapping — positional accuracy report
# ---------------------------------------------------------------------------


def _make_linestring(y: float, x_values: list[float]):
    """Lanelet2 LineString3d running along ``x_values`` at height ``y``."""
    import lanelet2

    points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), float(x), float(y), 0.0)
        for x in x_values
    ]
    return lanelet2.core.LineString3d(lanelet2.core.getId(), points)


def _make_linestring_xy(x_values: list[float], y_values: list[float]):
    """Lanelet2 LineString3d through explicit ``(x, y)`` pairs."""
    import lanelet2

    points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), float(x), float(y), 0.0)
        for x, y in zip(x_values, y_values)
    ]
    return lanelet2.core.LineString3d(lanelet2.core.getId(), points)


def _make_lanelet(left, right):
    """Convertible (``subtype=road``) lanelet from two boundary linestrings."""
    import lanelet2

    lanelet = lanelet2.core.Lanelet(lanelet2.core.getId(), left, right)
    lanelet.attributes["subtype"] = "road"
    return lanelet


def _straight_road_xml(
    road_id: int, y: float, length: float, lane_ids: list[int]
) -> str:
    """XODR ``<road>`` whose planView is a straight line at height ``y``."""
    lanes = "".join(f"<lane id='{lid}' type='driving'/>" for lid in lane_ids)
    return (
        f"<road id='{road_id}' junction='-1'><planView>"
        f"<geometry s='0.0' x='0.0' y='{y}' hdg='0.0' length='{length}'>"
        f"<line/></geometry></planView>"
        f"<lanes><laneSection s='0.0'><right>{lanes}</right></laneSection></lanes>"
        f"</road>"
    )


def _parse_roads(*road_xml: str) -> list[Road]:
    import lxml.etree as ET

    return parse_roads_from_xodr(
        Path("unused.xodr"),
        xodr_root=ET.fromstring("<OpenDRIVE>" + "".join(road_xml) + "</OpenDRIVE>"),
    )


class TestBuildMappingAccuracyReport:
    """``reference_match_distances`` must be an honest accuracy figure."""

    #: 20 m of road; boundaries sampled at the reference-line sample spacing
    #: so that these cases isolate one effect at a time.
    DENSE_X = [0.5 * i for i in range(41)]

    def test_perfect_fit_with_sparse_boundary_reports_zero(self) -> None:
        """A road lying exactly on a 5 m-spaced boundary reports ~0 m, not ~1.25 m."""
        import lanelet2

        sparse_x = [float(x) for x in range(0, 105, 5)]
        lanelet = _make_lanelet(
            _make_linestring(0.0, sparse_x), _make_linestring(-3.0, sparse_x)
        )
        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet)

        roads = _parse_roads(_straight_road_xml(1, 0.0, 100.0, [-1]))
        mapping = build_mapping(lanelet_map, roads, (0.0, 0.0), "x", "o")

        assert mapping.lanelet_to_road_and_lane == {lanelet.id: (1, -1)}
        assert mapping.reference_match_distances is not None
        record = mapping.reference_match_distances[lanelet.id]
        assert record.max_m == pytest.approx(0.0, abs=1e-6)
        assert record.mean_m == pytest.approx(0.0, abs=1e-6)
        assert record.boundary_length_m == pytest.approx(100.0)

    def test_rescued_road_is_included_in_the_report(self) -> None:
        """A road recovered by the rescue pass carries positional error too.

        Road 1 sits exactly on lanelet A; road 2 is within the matching
        tolerance of A as well, loses the conflict and is dropped, then the
        rescue pass recovers it on lanelet B at 4.6 m — beyond the 3.5 m
        matching tolerance.  Omitting it understates ``max`` and ``p95``.
        """
        import lanelet2

        lanelet_a = _make_lanelet(
            _make_linestring(0.0, self.DENSE_X), _make_linestring(-3.0, self.DENSE_X)
        )
        lanelet_b = _make_lanelet(
            _make_linestring(-8.0, self.DENSE_X), _make_linestring(-11.0, self.DENSE_X)
        )
        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet_a)
        lanelet_map.add(lanelet_b)

        roads = _parse_roads(
            _straight_road_xml(1, 0.0, 20.0, [-1]),
            _straight_road_xml(2, -3.4, 20.0, [-1]),
        )
        mapping = build_mapping(lanelet_map, roads, (0.0, 0.0), "x", "o")

        distances = mapping.reference_match_distances
        assert distances is not None
        assert distances[lanelet_a.id].max_m == pytest.approx(0.0, abs=1e-6)
        assert distances[lanelet_b.id].max_m == pytest.approx(4.6, abs=1e-6)
        assert mapping.rescued_reference_lanelets == [lanelet_b.id]

        summary = summarise_reference_match_accuracy(mapping)
        assert summary["count"] == 2
        assert summary["max_m"] == pytest.approx(4.6, abs=1e-6)
        assert summary["worst_lanelet_id"] == lanelet_b.id
        assert summary["rescued_count"] == 1
        assert summary["above_match_threshold"] == 1
        # Two 20 m boundaries matched -> 40 m; one road over each threshold.
        assert summary["matched_boundary_length_m"] == pytest.approx(40.0)
        assert summary["exceedances"][0]["per_km"] == pytest.approx(25.0)

    def test_distance_is_keyed_by_the_lanelet_the_walk_used(self) -> None:
        """When the pre-walk corrects the reference, the key follows it.

        The road's reference line is closest to the *inner* lanelet's left
        boundary, so Phase 1 picks lanelet 2.  The adjacency pre-walk then
        corrects the reference to lanelet 1 — the true edge lanelet — and the
        distance belongs to lanelet 1, measured against *its* boundary.
        """
        import lanelet2

        outer_left = _make_linestring(0.0, self.DENSE_X)
        shared = _make_linestring(-3.0, self.DENSE_X)
        inner_right = _make_linestring(-6.0, self.DENSE_X)
        lanelet_1 = _make_lanelet(outer_left, shared)
        lanelet_2 = _make_lanelet(shared, inner_right)
        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet_1)
        lanelet_map.add(lanelet_2)

        roads = _parse_roads(_straight_road_xml(1, -2.6, 20.0, [-1, -2]))
        mapping = build_mapping(lanelet_map, roads, (0.0, 0.0), "x", "o")

        assert mapping.lanelet_to_road_and_lane == {
            lanelet_1.id: (1, -1),
            lanelet_2.id: (1, -2),
        }
        distances = mapping.reference_match_distances
        assert distances is not None
        # Keyed by lanelet 1 (the reference the walk used), not lanelet 2
        # (the Phase-1 pick), and measured against lanelet 1's boundary.
        assert set(distances) == {lanelet_1.id}
        assert distances[lanelet_1.id].max_m == pytest.approx(2.6, abs=1e-6)

    def test_a_localised_spike_is_reported_by_the_max(self) -> None:
        """One bad vertex on an otherwise exact road must not be averaged away.

        The road lies exactly on its source boundary except for a single
        vertex pushed 1.5 m aside.  ``max_m`` reports that 1.5 m; ``mean_m``
        divides it by the road's whole length and reports centimetres, which
        is why it is only the secondary figure.
        """
        import lanelet2

        spike_m = 1.5
        left_x = list(self.DENSE_X)
        left_y = [-spike_m if x == 10.0 else 0.0 for x in left_x]
        left = _make_linestring_xy(left_x, left_y)
        lanelet = _make_lanelet(left, _make_linestring(-3.0, left_x))
        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet)

        roads = _parse_roads(_straight_road_xml(1, 0.0, 20.0, [-1]))
        mapping = build_mapping(lanelet_map, roads, (0.0, 0.0), "x", "o")

        assert mapping.lanelet_to_road_and_lane == {lanelet.id: (1, -1)}
        assert mapping.reference_match_distances is not None
        record = mapping.reference_match_distances[lanelet.id]
        assert record.max_m == pytest.approx(spike_m, abs=1e-6)
        assert record.mean_m < spike_m / 10.0

        # And the summary counts it as an exceedance rather than hiding it
        # behind a small median.
        summary = summarise_reference_match_accuracy(mapping)
        assert summary["max_m"] == pytest.approx(spike_m, abs=1e-6)
        assert summary["worst_lanelet_id"] == lanelet.id
        assert [exc["count"] for exc in summary["exceedances"]] == [1, 1]


# ---------------------------------------------------------------------------
# _compute_all_candidates — synthetic divergence connector handling (#493)
# ---------------------------------------------------------------------------


class TestComputeAllCandidatesSyntheticSkip:
    """#493: synthetic divergence/merge connecting roads (junction != -1,
    sub-0.5m total geometry) must be excluded from geometric matching *and*
    reported separately, not counted as 0-candidate matching failures."""

    def test_synthetic_connector_is_skipped_and_recorded(self) -> None:
        import lxml.etree as ET

        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            _compute_all_candidates,
        )

        # A degenerate divergence connecting road: junction != -1, total
        # planView length far below the synthetic-connector threshold.
        xodr = (
            "<OpenDRIVE><road id='100' junction='11000'><planView>"
            "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='0.05'>"
            "<line/></geometry></planView></road></OpenDRIVE>"
        )
        roads = parse_roads_from_xodr(
            Path("unused.xodr"), xodr_root=ET.fromstring(xodr)
        )
        all_rc, no_candidate_diag, skipped = _compute_all_candidates(
            roads, {}, {}, {}, {}
        )

        assert skipped == {100}
        assert 100 not in {rc.road_id for rc in all_rc}
        assert 100 not in no_candidate_diag

    def test_helper_identifies_synthetic_connectors_only(self) -> None:
        """_synthetic_connector_road_ids selects junction roads with sub-0.5m
        geometry, leaving real connecting roads and regular roads alone."""
        import lxml.etree as ET

        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            _synthetic_connector_road_ids,
        )

        xodr = (
            "<OpenDRIVE>"
            # synthetic connector: junction != -1, sub-0.5m total geometry
            "<road id='100' junction='11000'><planView>"
            "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='0.05'>"
            "<line/></geometry></planView></road>"
            # real connecting road: junction != -1 but several metres long
            "<road id='200' junction='1000'><planView>"
            "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='12.0'>"
            "<line/></geometry></planView></road>"
            # regular road: junction == -1 (short, but not a connector)
            "<road id='300' junction='-1'><planView>"
            "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='0.05'>"
            "<line/></geometry></planView></road>"
            "</OpenDRIVE>"
        )
        roads = parse_roads_from_xodr(
            Path("unused.xodr"), xodr_root=ET.fromstring(xodr)
        )
        assert _synthetic_connector_road_ids(roads) == {100}


# ---------------------------------------------------------------------------
# MappingMismatchError
# ---------------------------------------------------------------------------


class TestMappingMismatchError:
    def test_is_exception(self) -> None:
        assert issubclass(MappingMismatchError, Exception)

    def test_can_be_raised_and_caught(self) -> None:
        with pytest.raises(MappingMismatchError):
            raise MappingMismatchError("test message")


# ---------------------------------------------------------------------------
# _resolve_conflicts
# ---------------------------------------------------------------------------


def _make_rc(
    road_id: int,
    candidates: list[tuple[float, int]],
    raw_dists: dict[int, float] | None = None,
) -> _RoadCandidates:
    """Helper to build a minimal _RoadCandidates for conflict-resolution tests."""
    return _RoadCandidates(
        road=MagicMock(),
        road_id=road_id,
        lane_ids=[-1],
        is_rht=True,
        ref_line=MagicMock(),
        candidates=candidates,
        raw_dists=raw_dists or {lid: dist for dist, lid in candidates},
        walk_lane_ids=[-1],
    )


class TestResolveConflicts:
    """Unit tests for the _resolve_conflicts function."""

    def test_no_conflict(self) -> None:
        """Two roads claim different lanelets — both keep index 0."""
        rc_a = _make_rc(100, [(1.0, 10)])
        rc_b = _make_rc(200, [(2.0, 20)])
        result = _resolve_conflicts([rc_a, rc_b])
        assert result == {0: 0, 1: 0}

    def test_2way_cost_advance_loser(self) -> None:
        """Greedy loser has a cheap alternative — advance loser (same as greedy)."""
        # Both claim lanelet 10. Road 100 is closer (1.0) than 200 (2.0).
        # Loser (200) next candidate: (3.0, 20). Winner (100) next: (10.0, 30).
        # cost_advance_loser = 1.0 + 3.0 = 4.0
        # cost_advance_winner = 2.0 + 10.0 = 12.0
        # Advance loser is cheaper -> advance loser.
        rc_a = _make_rc(100, [(1.0, 10), (10.0, 30)])
        rc_b = _make_rc(200, [(2.0, 10), (3.0, 20)])
        result = _resolve_conflicts([rc_a, rc_b])
        # Road 100 keeps lanelet 10 (idx 0), road 200 advances to lanelet 20 (idx 1)
        assert result == {0: 0, 1: 1}

    def test_2way_cost_advance_winner(self) -> None:
        """Winner has a cheap alternative — cost-aware advances winner instead."""
        # Both claim lanelet 10. Road 100 is closer (1.0) than 200 (2.0).
        # Loser (200) next candidate: (20.0, 20). Winner (100) next: (1.5, 30).
        # cost_advance_loser = 1.0 + 20.0 = 21.0
        # cost_advance_winner = 2.0 + 1.5 = 3.5
        # Advance winner is cheaper -> advance winner (differs from greedy).
        rc_a = _make_rc(100, [(1.0, 10), (1.5, 30)])
        rc_b = _make_rc(200, [(2.0, 10), (20.0, 20)])
        result = _resolve_conflicts([rc_a, rc_b])
        # Road 100 advances to lanelet 30 (idx 1), road 200 keeps lanelet 10 (idx 0)
        assert result == {0: 1, 1: 0}

    def test_save_the_drowning(self) -> None:
        """Loser exhausted, winner has alt — advance winner to save loser."""
        rc_a = _make_rc(100, [(1.0, 10), (5.0, 30)])
        rc_b = _make_rc(200, [(2.0, 10)])  # only one candidate
        result = _resolve_conflicts([rc_a, rc_b])
        # Winner (100) advances to 30, loser (200) keeps 10
        assert result == {0: 1, 1: 0}

    def test_both_exhausted_loser_dropped(self) -> None:
        """Both have only one candidate for the same lanelet — loser is dropped."""
        rc_a = _make_rc(100, [(1.0, 10)])
        rc_b = _make_rc(200, [(2.0, 10)])
        result = _resolve_conflicts([rc_a, rc_b])
        # Winner (100) keeps 10, loser (200) is dropped
        assert result == {0: 0}
        assert 1 not in result

    def test_multi_way_conflict(self) -> None:
        """Three roads claim the same lanelet — all losers advance."""
        rc_a = _make_rc(100, [(1.0, 10), (5.0, 30)])
        rc_b = _make_rc(200, [(2.0, 10), (6.0, 40)])
        rc_c = _make_rc(300, [(3.0, 10), (7.0, 50)])
        result = _resolve_conflicts([rc_a, rc_b, rc_c])
        # Winner (100) keeps lanelet 10, losers advance
        assert result[0] == 0  # road 100 -> lanelet 10
        assert result[1] == 1  # road 200 -> lanelet 40
        assert result[2] == 1  # road 300 -> lanelet 50

    def test_chain_conflict(self) -> None:
        """Advancing a loser creates a new conflict resolved in next iteration."""
        # Road 100 and 200 both claim lanelet 10.
        # Road 200 loses, advances to lanelet 20.
        # Road 300 also claims lanelet 20 — new conflict.
        rc_a = _make_rc(100, [(1.0, 10)])
        rc_b = _make_rc(200, [(2.0, 10), (3.0, 20)])
        rc_c = _make_rc(300, [(2.5, 20), (4.0, 30)])
        result = _resolve_conflicts([rc_a, rc_b, rc_c])
        # Road 100 -> lanelet 10 (idx 0)
        assert result[0] == 0
        # Road 200 advanced to lanelet 20 (idx 1), then conflicts with 300.
        # 300 is closer to 20 (2.5 < 3.0), so 200 loses again -> advances to next
        # But road 200 has no 3rd candidate, so check cost or exhaustion.
        # 200's next after idx 1 = idx 2 = out of range -> exhausted.
        # 300's next after idx 0 = idx 1 -> has alt.
        # Save the drowning: advance 300 to idx 1 (lanelet 30).
        assert result[1] == 1  # road 200 -> lanelet 20
        assert result[2] == 1  # road 300 -> lanelet 30

    def test_winner_exhausted_loser_has_alt(self) -> None:
        """Winner exhausted but loser has alternatives — advance loser."""
        rc_a = _make_rc(100, [(1.0, 10)])  # winner, no alternatives
        rc_b = _make_rc(200, [(2.0, 10), (3.0, 20)])  # loser, has alt
        result = _resolve_conflicts([rc_a, rc_b])
        # Winner (100) has no alt, loser (200) has alt -> advance loser
        assert result == {0: 0, 1: 1}


# ---------------------------------------------------------------------------
# _resolve_conflicts: pairwise swap search
# ---------------------------------------------------------------------------


def _exhaustive_swap_fix(
    all_rc: list[_RoadCandidates], valid: dict[int, int]
) -> dict[int, int]:
    """Reference swap search: the every-pair scan the indexed one replaces.

    Kept verbatim (bar the debug log) so the production search can be checked
    against it for both result and cost.  It is the definition of correct
    here: the two must pick the same swaps, in the same order.
    """
    valid = dict(valid)
    swap_found = True
    while swap_found:
        swap_found = False
        items = list(valid.items())
        for i in range(len(items)):
            rc_a, cand_a = items[i]
            _, lid_a = all_rc[rc_a].candidates[cand_a]
            raw_a = all_rc[rc_a].raw_dists.get(lid_a, float("inf"))
            for j in range(i + 1, len(items)):
                rc_b, cand_b = items[j]
                _, lid_b = all_rc[rc_b].candidates[cand_b]
                raw_b = all_rc[rc_b].raw_dists.get(lid_b, float("inf"))

                raw_a_new = all_rc[rc_a].raw_dists.get(lid_b)
                if raw_a_new is None:
                    continue
                raw_b_new = all_rc[rc_b].raw_dists.get(lid_a)
                if raw_b_new is None:
                    continue

                if raw_a_new + raw_b_new < raw_a + raw_b:
                    valid[rc_a] = next(
                        ci
                        for ci, (_, cand_lid) in enumerate(all_rc[rc_a].candidates)
                        if cand_lid == lid_b
                    )
                    valid[rc_b] = next(
                        ci
                        for ci, (_, cand_lid) in enumerate(all_rc[rc_b].candidates)
                        if cand_lid == lid_a
                    )
                    swap_found = True
                    break
            if swap_found:
                break
    return valid


def _conflict_free_population(
    rng: np.random.Generator, count: int, extras: int, pool: int
) -> list[_RoadCandidates]:
    """Roads whose *top* candidates are all distinct, so no road is in conflict.

    The conflict loop then leaves every road on candidate 0 and the swap
    search is the only thing that runs, which is what these tests measure.
    Lower-ranked candidates are drawn from a shared pool, so roads do overlap
    there and swaps are possible.
    """
    all_rc: list[_RoadCandidates] = []
    for index in range(count):
        top = (1.0 + float(rng.uniform(0.0, 0.5)), index)
        others = {
            int(rng.integers(count, count + pool)): 1.5 + float(rng.uniform(0.0, 1.0))
            for _ in range(extras)
        }
        candidates = sorted([top] + [(dist, lid) for lid, dist in others.items()])
        all_rc.append(_make_rc(1000 + index, candidates))
    return all_rc


class TestSwapSearch:
    """The swap search finds pairs through a candidate index instead of
    scanning every later road.  It must pick the same swaps as the exhaustive
    scan, and it must stop being quadratic in the road count."""

    def test_swap_is_applied_when_it_lowers_total_distance(self) -> None:
        """Two roads each holding the other's nearer lanelet are swapped."""
        # Greedy gives 100->10 and 200->20 (no conflict: different top
        # candidates), but 100 is nearer 20 and 200 is nearer 10.
        rc_a = _make_rc(100, [(1.0, 10), (2.0, 20)], raw_dists={10: 5.0, 20: 0.5})
        rc_b = _make_rc(200, [(1.5, 20), (3.0, 10)], raw_dists={20: 5.0, 10: 0.5})
        assert _resolve_conflicts([rc_a, rc_b]) == {0: 1, 1: 1}

    def test_no_swap_when_it_would_not_help(self) -> None:
        rc_a = _make_rc(100, [(1.0, 10), (2.0, 20)], raw_dists={10: 0.5, 20: 5.0})
        rc_b = _make_rc(200, [(1.5, 20), (3.0, 10)], raw_dists={20: 0.5, 10: 5.0})
        assert _resolve_conflicts([rc_a, rc_b]) == {0: 0, 1: 0}

    def test_matches_the_exhaustive_scan(self) -> None:
        """Same assignment as the every-pair reference, over random inputs."""
        rng = np.random.default_rng(20260920)
        for _ in range(40):
            all_rc = _conflict_free_population(rng, count=60, extras=3, pool=20)
            expected = _exhaustive_swap_fix(
                all_rc, {index: 0 for index in range(len(all_rc))}
            )
            assert _resolve_conflicts(all_rc) == expected

    def test_is_no_longer_quadratic(self) -> None:
        """The indexed search must beat the exhaustive one by a wide margin.

        A ratio rather than an absolute wall-clock, so a loaded runner slows
        both sides.  The two are timed **interleaved** and the **fastest**
        sample of each is compared: timing one side to completion and then the
        other lets a load spike land entirely on one of them (see
        ``test_issue_registration_is_no_longer_quadratic``).
        """
        rng = np.random.default_rng(11)
        count = 2000
        rounds = 3
        all_rc = _conflict_free_population(rng, count=count, extras=2, pool=40)
        start = {index: 0 for index in range(len(all_rc))}

        indexed_samples: list[float] = []
        exhaustive_samples: list[float] = []
        for _ in range(rounds):
            t0 = time.perf_counter()
            indexed = _resolve_conflicts(all_rc)
            indexed_samples.append(time.perf_counter() - t0)

            t0 = time.perf_counter()
            exhaustive = _exhaustive_swap_fix(all_rc, start)
            exhaustive_samples.append(time.perf_counter() - t0)

        assert indexed == exhaustive
        fastest_indexed = min(indexed_samples)
        fastest_exhaustive = min(exhaustive_samples)
        assert fastest_indexed * 4 < fastest_exhaustive, (
            f"indexed={fastest_indexed:.3f}s exhaustive={fastest_exhaustive:.3f}s "
            f"for {count} roads (best of {rounds}; "
            f"indexed={[round(v, 3) for v in indexed_samples]} "
            f"exhaustive={[round(v, 3) for v in exhaustive_samples]})"
        )


# ---------------------------------------------------------------------------
# _sample_reference_line_from_road  (issue #495)
# ---------------------------------------------------------------------------


class TestSampleReferenceLineFromRoad:
    """A road's planView must be sampled with each geometry's analytic model.

    Regression test for #495: with arc-primitive detection enabled, an
    ``<arc>`` segment must be sampled along its curve, not as the straight
    chord between its endpoints — chord sampling inflated road↔lanelet
    distances and aborted conversion with a ``MappingMismatchError``.
    """

    def test_arc_geometry_is_sampled_along_its_curve(self) -> None:
        # Quarter circle of radius 10 m: starts at the origin heading +x,
        # curves left (positive curvature), ends at (10, 10) heading +y.
        radius = 10.0
        curvature = 1.0 / radius
        length = radius * math.pi / 2.0
        road = Road(
            id=1,
            plan_view=PlanView(
                geometries=[
                    Arc(
                        s=0.0,
                        x=0.0,
                        y=0.0,
                        hdg=0.0,
                        length=length,
                        curvature=curvature,
                    )
                ]
            ),
        )

        # spacing 2.0 m over the 15.708 m quarter circle -> 8 intervals.
        pts = _sample_reference_line_from_road(road, sample_spacing=2.0)

        # Every sample must lie on the analytic arc, not the straight chord
        # (chord sampling keeps y == 0, which is the bug being fixed).
        for i in range(8):
            p = length * i / 8
            expected = (
                math.sin(curvature * p) / curvature,
                (1.0 - math.cos(curvature * p)) / curvature,
            )
            assert pts[i] == pytest.approx(expected, abs=1e-6)
        # Closing endpoint: the quarter circle ends exactly at (radius, radius).
        assert pts[-1] == pytest.approx((radius, radius), abs=1e-6)

    def test_line_geometry_is_sampled_as_a_straight_line(self) -> None:
        # A 20 m line heading 30 degrees, starting at (1, 2).
        hdg = math.radians(30.0)
        length = 20.0
        road = Road(
            id=2,
            plan_view=PlanView(
                geometries=[Line(s=0.0, x=1.0, y=2.0, hdg=hdg, length=length)]
            ),
        )

        # spacing 5.0 m over the 20 m line -> 4 intervals.
        pts = _sample_reference_line_from_road(road, sample_spacing=5.0)

        for i in range(4):
            p = length * i / 4
            expected = (1.0 + p * math.cos(hdg), 2.0 + p * math.sin(hdg))
            assert pts[i] == pytest.approx(expected, abs=1e-6)
        assert pts[-1] == pytest.approx(
            (1.0 + length * math.cos(hdg), 2.0 + length * math.sin(hdg)),
            abs=1e-6,
        )

    def test_param_poly3_geometry_is_sampled_with_cubic_model(self) -> None:
        # A paramPoly3 heading +x from the origin with a quadratic lateral
        # offset v = 0.1 * u**2.
        length = 10.0
        road = Road(
            id=3,
            plan_view=PlanView(
                geometries=[
                    ParamPoly3(
                        s=0.0,
                        x=0.0,
                        y=0.0,
                        hdg=0.0,
                        length=length,
                        aU=0.0,
                        bU=1.0,
                        cU=0.0,
                        dU=0.0,
                        aV=0.0,
                        bV=0.0,
                        cV=0.1,
                        dV=0.0,
                    )
                ]
            ),
        )

        # spacing 2.0 m over the 10 m paramPoly3 -> 5 intervals.
        pts = _sample_reference_line_from_road(road, sample_spacing=2.0)

        for i in range(5):
            p = length * i / 5
            assert pts[i] == pytest.approx((p, 0.1 * p * p), abs=1e-6)
        assert pts[-1] == pytest.approx((length, 0.1 * length * length), abs=1e-6)

    def test_sampling_is_uniform_across_uneven_segments(self) -> None:
        """Samples are spaced uniformly by arc-length regardless of how the
        planView is split into segments (#499).

        Arc-primitive detection re-segments a road into a few long arcs plus
        short paramPoly3 runs. A fixed sample count per segment would bunch
        points onto the short segments and skew the mean-distance metric used
        for road-lanelet matching, flipping which lanelet a road maps to.
        Two collinear straight segments of very different lengths must still
        yield evenly-spaced samples.
        """
        # A 4 m line followed by a collinear 16 m line (total 20 m, heading +x).
        road = Road(
            id=1,
            plan_view=PlanView(
                geometries=[
                    Line(s=0.0, x=0.0, y=0.0, hdg=0.0, length=4.0),
                    Line(s=4.0, x=4.0, y=0.0, hdg=0.0, length=16.0),
                ]
            ),
        )

        pts = _sample_reference_line_from_road(road, sample_spacing=1.0)

        # 1.0 m spacing over 20 m -> 20 intervals, 21 points, uniform.
        assert len(pts) == 21
        spacings = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        assert spacings.max() - spacings.min() < 1e-9
        assert spacings[0] == pytest.approx(1.0)
        assert pts[0] == pytest.approx((0.0, 0.0), abs=1e-9)
        assert pts[-1] == pytest.approx((20.0, 0.0), abs=1e-9)


# ---------------------------------------------------------------------------
# parse_roads_from_xodr  (issue #502)
# ---------------------------------------------------------------------------


class TestParseRoadsFromXodr:
    """parse_roads_from_xodr must reconstruct every planView primitive.

    Regression test for #502: the XODR re-parser used by the analyze/QC
    path previously handled only ``<paramPoly3>`` and silently dropped
    ``<arc>`` and ``<line>`` geometry, breaking validation of maps
    converted with arc-primitive detection enabled.
    """

    def test_parses_line_arc_and_param_poly3(self) -> None:
        import lxml.etree as ET

        xodr = (
            "<OpenDRIVE><road id='7' junction='-1'><planView>"
            "<geometry s='0.0' x='1.0' y='2.0' hdg='0.5' length='10.0'>"
            "<line/></geometry>"
            "<geometry s='10.0' x='3.0' y='4.0' hdg='0.6' length='20.0'>"
            "<arc curvature='0.04'/></geometry>"
            "<geometry s='30.0' x='5.0' y='6.0' hdg='0.7' length='8.0'>"
            "<paramPoly3 aU='0.0' bU='1.0' cU='0.0' dU='0.0'"
            " aV='0.0' bV='0.0' cV='0.1' dV='0.0'/></geometry>"
            "</planView></road></OpenDRIVE>"
        )
        roads = parse_roads_from_xodr(
            Path("unused.xodr"), xodr_root=ET.fromstring(xodr)
        )

        assert len(roads) == 1
        plan_view = roads[0].plan_view
        assert plan_view is not None
        geometries = plan_view.geometries
        assert [type(g).__name__ for g in geometries] == [
            "Line",
            "Arc",
            "ParamPoly3",
        ]

        line, arc, pp3 = geometries
        assert isinstance(line, Line)
        assert (line.s, line.x, line.y, line.hdg, line.length) == (
            0.0,
            1.0,
            2.0,
            0.5,
            10.0,
        )
        assert isinstance(arc, Arc)
        assert arc.length == pytest.approx(20.0)
        assert arc.curvature == pytest.approx(0.04)
        assert isinstance(pp3, ParamPoly3)
        assert pp3.cV == pytest.approx(0.1)


# ---------------------------------------------------------------------------
# Boundary grid pre-filter
# ---------------------------------------------------------------------------


class TestBoundaryGridPreFilter:
    """The grid in front of the Phase-1 bbox test is a filter only: it must
    return every boundary the exhaustive scan would have kept, in the same
    order, so the candidate set and the nearest-rejected diagnostic are
    unchanged."""

    @staticmethod
    def _population(
        rng: np.random.Generator, count: int, extent: float
    ) -> tuple[dict[int, np.ndarray], dict[int, tuple[float, float, float, float]]]:
        boundaries: dict[int, np.ndarray] = {}
        bboxes: dict[int, tuple[float, float, float, float]] = {}
        for index in range(count):
            start = rng.uniform(-extent, extent, size=2)
            delta = rng.uniform(-40.0, 40.0, size=2)
            pts = np.array([start, start + delta * 0.5, start + delta])
            lid = 1000 + index * 7
            boundaries[lid] = pts
            bboxes[lid] = _bbox(pts)
        return boundaries, bboxes

    def test_grid_matches_exhaustive_scan(self) -> None:
        rng = np.random.default_rng(20240513)
        boundaries, bboxes = self._population(rng, 200, 300.0)
        grid = _build_boundary_grid(boundaries, bboxes)

        for _ in range(200):
            start = rng.uniform(-300.0, 300.0, size=2)
            delta = rng.uniform(-60.0, 60.0, size=2)
            ref_bbox = _bbox(np.array([start, start + delta]))

            exhaustive = [
                lid
                for lid in boundaries
                if _bboxes_overlap(ref_bbox, bboxes[lid], margin=_CANDIDATE_BBOX_MARGIN)
            ]
            filtered = [
                grid.lids[position]
                for position in _grid_candidate_positions(
                    grid, ref_bbox, _CANDIDATE_BBOX_MARGIN
                )
                if _bboxes_overlap(
                    ref_bbox, grid.boxes[position], margin=_CANDIDATE_BBOX_MARGIN
                )
            ]
            assert filtered == exhaustive

    def test_grid_query_is_a_superset_of_the_exact_test(self) -> None:
        """The cell sweep alone — before the exact test — already contains
        every overlapping boundary."""
        rng = np.random.default_rng(7)
        boundaries, bboxes = self._population(rng, 120, 150.0)
        grid = _build_boundary_grid(boundaries, bboxes)

        for _ in range(100):
            start = rng.uniform(-150.0, 150.0, size=2)
            ref_bbox = _bbox(
                np.array([start, start + rng.uniform(-50.0, 50.0, size=2)])
            )
            swept = {
                grid.lids[position]
                for position in _grid_candidate_positions(
                    grid, ref_bbox, _CANDIDATE_BBOX_MARGIN
                )
            }
            overlapping = {
                lid
                for lid in boundaries
                if _bboxes_overlap(ref_bbox, bboxes[lid], margin=_CANDIDATE_BBOX_MARGIN)
            }
            assert overlapping <= swept

    def test_margin_covers_the_rescue_threshold(self) -> None:
        """The filter margin has to sit above every distance threshold that a
        surviving pair is later measured against, or a real match could be
        discarded before it is ever measured."""
        rescue_threshold = (
            DEFAULT_CONFIG.geo_mapping.match_threshold
            * DEFAULT_CONFIG.geo_mapping.rescue_threshold_factor
        )
        assert _CANDIDATE_BBOX_MARGIN > rescue_threshold

    def test_empty_population(self) -> None:
        grid = _build_boundary_grid({}, {})
        assert grid.lids == []
        assert _grid_candidate_positions(grid, (0.0, 0.0, 10.0, 10.0), 10.0) == []

    def test_zero_extent_boundaries_do_not_collapse_the_grid(self) -> None:
        """A population of coincident points has no extent; the cell size
        falls back to the configured floor rather than zero."""
        boundaries = {1: np.array([[5.0, 5.0], [5.0, 5.0]])}
        bboxes = {1: _bbox(boundaries[1])}
        grid = _build_boundary_grid(boundaries, bboxes)

        assert grid.cell_size == DEFAULT_CONFIG.geometry.spatial_grid_min_cell_size
        assert (
            grid.lids[_grid_candidate_positions(grid, (0.0, 0.0, 1.0, 1.0), 10.0)[0]]
            == 1
        )


class TestAccuracyReportIsOptIn:
    """The report must be switchable off without editing the source.

    It adds a section to ``.mapping.json`` on every conversion and can warn
    about maps that are fine, so a pipeline that does not want it needs a way
    to say so.
    """

    DENSE_X = [0.5 * i for i in range(41)]

    def _convert(self, tmp_path: Path, report_accuracy: bool = True) -> dict:
        """Run the save/validate entry point and return the mapping JSON."""
        import lanelet2

        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            validate_and_save_mapping,
        )

        lanelet = _make_lanelet(
            _make_linestring(0.0, self.DENSE_X), _make_linestring(-3.0, self.DENSE_X)
        )
        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet)

        xodr_path = tmp_path / "test.xodr"
        xodr_xml = _straight_road_xml(1, 0.0, 20.0, [-1])
        xodr_path.write_text(xodr_xml, encoding="utf-8")
        osm_path = tmp_path / "test.osm"
        osm_path.write_text("<osm/>", encoding="utf-8")

        result = validate_and_save_mapping(
            lanelet_to_road_and_lane={lanelet.id: (1, -1)},
            lanelet_map=lanelet_map,
            roads=_parse_roads(xodr_xml),
            xodr_path=xodr_path,
            osm_path=osm_path,
            mgrs_offset=(0.0, 0.0),
            strict=False,
            report_accuracy=report_accuracy,
        )
        return json.loads(result.read_text(encoding="utf-8"))

    def test_report_is_written_by_default(self, tmp_path: Path) -> None:
        data = self._convert(tmp_path)
        assert "reference_match_distances" in data

    def test_report_can_be_switched_off(self, tmp_path: Path) -> None:
        data = self._convert(tmp_path, report_accuracy=False)
        assert "reference_match_distances" not in data, (
            "the positional-accuracy report cannot be switched off, so it "
            "changes .mapping.json on every conversion"
        )


class TestOriginalLaneletIds:
    """A reported lanelet id is a preprocessed id, and must be traceable back.

    ``main`` converts the *preprocessed* map, so a lanelet that preprocessing
    merged carries an id that does not appear in the file the user supplied.
    Reporting such an id without a way back makes the worst-offender list
    unusable: there is nothing to look up in the source map.
    """

    def _mapping_with_merge(self) -> GeoRoadLaneletMapping:
        """Mapping whose lanelet 99 was merged from input lanelets 1 and 2."""
        return GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={99: (1, -1)},
            preprocessing_log={
                "config_path": None,
                "entries": [
                    {
                        "operation": "merge",
                        "input_ids": [1, 2],
                        "output_ids": [99],
                        "details": "merged 1 and 2",
                    }
                ],
            },
        )

    @pytest.mark.xfail(
        strict=True,
        reason="preprocessed ids are reported with no way back (awf#14 review, finding 4)",
    )
    def test_a_merged_id_resolves_to_the_ids_it_was_made_from(self) -> None:
        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            original_lanelet_ids,
        )

        assert original_lanelet_ids(self._mapping_with_merge(), 99) == [1, 2]

    def test_an_untouched_id_resolves_to_itself(self) -> None:
        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            original_lanelet_ids,
        )

        assert original_lanelet_ids(self._mapping_with_merge(), 7) == [7]

    @pytest.mark.xfail(
        strict=True,
        reason="preprocessed ids are reported with no way back (awf#14 review, finding 4)",
    )
    def test_a_chain_of_operations_is_followed_to_the_input(self) -> None:
        """A lanelet merged twice must resolve to the original input ids."""
        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            original_lanelet_ids,
        )

        mapping = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={100: (1, -1)},
            preprocessing_log={
                "config_path": None,
                "entries": [
                    {
                        "operation": "merge",
                        "input_ids": [1, 2],
                        "output_ids": [99],
                        "details": "",
                    },
                    {
                        "operation": "merge",
                        "input_ids": [99, 3],
                        "output_ids": [100],
                        "details": "",
                    },
                ],
            },
        )
        assert original_lanelet_ids(mapping, 100) == [1, 2, 3]


class TestLocalisedSpikeIsNotDiluted:
    """A road that is wrong in one spot must be reported at full size.

    The mean divides that one spot by the whole length of the road, so the
    same defect reports ten times smaller on a road ten times longer.  The
    maximum is what the accuracy figure is for.
    """

    SPIKE_M = 1.5
    SPIKE_AT_X = 10.0

    def _case(self, length_m: float) -> tuple[np.ndarray, np.ndarray]:
        """Straight reference line vs. a boundary with one displaced vertex."""
        x_values = [0.5 * i for i in range(int(length_m / 0.5) + 1)]
        ref_line = _polyline(0.0, x_values)
        boundary = np.array(
            [[x, -self.SPIKE_M if x == self.SPIKE_AT_X else 0.0] for x in x_values]
        )
        return ref_line, boundary

    def test_max_is_independent_of_road_length(self) -> None:
        for length_m in (20.0, 200.0):
            assert _symmetric_max_polyline_distance(
                *self._case(length_m)
            ) == pytest.approx(self.SPIKE_M)

    def test_mean_shrinks_as_the_road_grows(self) -> None:
        """The reason the mean cannot be the headline figure."""
        short = _symmetric_mean_polyline_distance(*self._case(20.0))
        long = _symmetric_mean_polyline_distance(*self._case(200.0))
        assert long < short / 5.0

    def test_mean_understates_the_spike_by_an_order_of_magnitude(self) -> None:
        mean = _symmetric_mean_polyline_distance(*self._case(20.0))
        assert mean < self.SPIKE_M / 10.0
