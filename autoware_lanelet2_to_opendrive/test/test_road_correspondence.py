"""Tests for matching the roads of two conversions by their source lanelets.

Road ids are reassigned whenever a change regroups roads, so two outputs of the
same map cannot be compared road id against road id. These tests pin the key
that does survive: the set of source lanelets a road was built from.

The mapping documents are hand-written and minimal. No map is converted here:
the correspondence is a function of two ``.mapping.json`` documents only.
"""

from __future__ import annotations

import json
import sys

import pytest

from autoware_lanelet2_to_opendrive import xodr_digest
from autoware_lanelet2_to_opendrive.road_correspondence import match_roads

pytestmark = pytest.mark.xfail(
    strict=True,
    reason="road correspondence is a stub: match_roads() returns an empty result",
)


def _mapping(lanelet_to_road: dict[int, int], **extra) -> dict:
    """A mapping document with every lanelet on lane -1 of its road."""
    return {
        "xodr_sha256": "0" * 64,
        "osm_sha256": "0" * 64,
        "lanelet_to_road_and_lane": {
            str(lanelet): [road, -1] for lanelet, road in lanelet_to_road.items()
        },
        **extra,
    }


def _merge_log(*merges: tuple[list[int], int]) -> dict:
    return {
        "config_path": "conf/map/test.yaml",
        "entries": [
            {
                "operation": "merge",
                "input_ids": inputs,
                "output_ids": [output],
                "details": f"Merged {len(inputs)} lanelets {inputs} into lanelet {output}",
            }
            for inputs, output in merges
        ],
    }


def test_unchanged_grouping_is_the_identity_map():
    """Two conversions with the same road composition match road for road."""
    mapping = _mapping({100: 0, 101: 0, 102: 1, 103: 2})

    result = match_roads(mapping, mapping)

    assert {
        road: entry["best_match"]
        for road, entry in result["baseline_to_comparison"].items()
    } == {"0": "0", "1": "1", "2": "2"}
    assert all(
        entry["jaccard"] == 1.0 and entry["symmetric"]
        for entry in result["baseline_to_comparison"].values()
    )
    assert result["summary"]["is_identity_map"] is True
    assert result["unmatched_baseline"] == []
    assert result["new_in_comparison"] == []


def test_renumbered_roads_are_matched_by_what_they_are_built_from():
    """A road keeps its identity when a regrouping elsewhere shifts its id."""
    baseline = _mapping({100: 0, 101: 1, 102: 2})
    # Lanelet 99 now forms a new road 0; every other road moves up by one.
    comparison = _mapping({99: 0, 100: 1, 101: 2, 102: 3})

    result = match_roads(baseline, comparison)

    assert {
        road: entry["best_match"]
        for road, entry in result["baseline_to_comparison"].items()
    } == {"0": "1", "1": "2", "2": "3"}
    assert result["summary"]["is_identity_map"] is False
    assert result["new_in_comparison"] == ["0"]


def test_a_merged_lanelet_is_matched_through_the_preprocessing_log():
    """A ``merge_operations`` entry must not make the merged road unmatchable.

    Merging replaces the listed lanelets by one lanelet under a new synthetic
    id, so the comparison names an id the baseline has never heard of. The
    mapping records the merge in ``preprocessing_log``; reading it restores the
    shared key. Measured on Nishi-Shinjuku without this: 6 of 640 roads
    unmatched, exactly the 6 ``merge_operations`` entries.
    """
    baseline = _mapping({112: 10, 114: 11, 200: 12})
    comparison = _mapping(
        {3013189: 10, 200: 11},
        preprocessing_log=_merge_log(([112, 114], 3013189)),
    )

    result = match_roads(baseline, comparison)

    matches = result["baseline_to_comparison"]
    assert set(matches) == {"10", "11", "12"}
    assert result["unmatched_baseline"] == []
    assert result["new_in_comparison"] == []
    assert matches["10"]["best_match"] == "10"
    assert matches["11"]["best_match"] == "10"
    assert matches["10"]["jaccard"] == pytest.approx(0.5)
    assert matches["12"]["best_match"] == "11"
    assert matches["12"]["jaccard"] == 1.0
    # Two baseline roads became one: reported, not silently collapsed.
    assert result["many_to_one"] == [["10", "11"]]


def test_a_merge_of_a_merge_is_expanded_to_the_original_lanelets():
    """A synthetic id that is itself merged again still resolves to the originals."""
    baseline = _mapping({1: 0, 2: 0, 3: 0})
    comparison = _mapping(
        {9002: 0},
        preprocessing_log=_merge_log(([1, 2], 9001), ([9001, 3], 9002)),
    )

    result = match_roads(baseline, comparison)

    assert set(result["baseline_to_comparison"]) == {"0"}
    assert result["baseline_to_comparison"]["0"]["best_match"] == "0"
    assert result["baseline_to_comparison"]["0"]["jaccard"] == 1.0


def test_roads_without_a_source_lanelet_are_reported_not_dropped():
    """Synthetic connectors have nothing to match on, and the result says so.

    A correspondence that silently omitted them would read as "every road was
    compared" when a class of roads was never looked at.
    """
    baseline = _mapping({100: 0}, skipped_synthetic_roads=[7, 8])
    comparison = _mapping({100: 0}, skipped_synthetic_roads=[7, 8, 9])

    result = match_roads(baseline, comparison)

    assert result["not_matchable"] == {
        "baseline": ["7", "8"],
        "comparison": ["7", "8", "9"],
    }
    assert result["summary"]["not_matchable_baseline"] == 2
    assert result["summary"]["not_matchable_comparison"] == 3
    assert "7" not in result["baseline_to_comparison"]


def test_a_lanelet_emitted_on_several_roads_counts_for_each_of_them():
    """The extra slots a mapping records are part of a road's composition."""
    baseline = _mapping({100: 0, 101: 1})
    comparison = _mapping(
        {100: 0, 101: 1},
        lanelet_additional_road_lanes={"100": [[5, -1]]},
        exit_fork_duplicate_roads={"6": [100]},
        chain_merged_roads={"1": [101, 102]},
    )

    result = match_roads(baseline, comparison)

    # Roads 5 and 6 are further copies of lanelet 100's stage: one baseline
    # road now appears on three, and none of them is a road nobody has seen.
    assert result["one_to_many"] == [["0", ["0", "5", "6"]]]
    assert result["new_in_comparison"] == []
    assert result["baseline_to_comparison"]["0"]["best_match"] == "0"
    # Road 1 spans a chain of two lanelets, which the 1:1 field cannot say.
    assert result["baseline_to_comparison"]["1"]["comparison_lanelets"] == 2
    assert result["baseline_to_comparison"]["1"]["jaccard"] == pytest.approx(0.5)


def test_cli_writes_the_correspondence(tmp_path, monkeypatch, capsys):
    """``xodr-digest --match-roads`` compares two mapping files."""
    baseline_path = tmp_path / "baseline.mapping.json"
    comparison_path = tmp_path / "comparison.mapping.json"
    baseline_path.write_text(json.dumps(_mapping({112: 10, 114: 11})), encoding="utf-8")
    comparison_path.write_text(
        json.dumps(
            _mapping(
                {3013189: 10},
                preprocessing_log=_merge_log(([112, 114], 3013189)),
            )
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["xodr-digest", "--match-roads", str(baseline_path), str(comparison_path)],
    )

    xodr_digest.main()

    result = json.loads(capsys.readouterr().out)
    assert result["summary"]["baseline_roads"] == 2
    assert result["summary"]["comparison_roads"] == 1
    assert result["unmatched_baseline"] == []
