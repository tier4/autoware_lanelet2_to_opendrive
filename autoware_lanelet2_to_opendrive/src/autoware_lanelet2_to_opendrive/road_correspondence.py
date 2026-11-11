"""Match the roads of two conversions of the same map by their source lanelets."""

from __future__ import annotations

from typing import Any


def match_roads(
    baseline_mapping: dict[str, Any], comparison_mapping: dict[str, Any]
) -> dict[str, Any]:
    """Return the road correspondence between two mapping documents."""
    return {
        "baseline_to_comparison": {},
        "unmatched_baseline": [],
        "new_in_comparison": [],
        "many_to_one": [],
        "one_to_many": [],
        "not_matchable": {"baseline": [], "comparison": []},
        "summary": {
            "baseline_roads": 0,
            "comparison_roads": 0,
            "not_matchable_baseline": 0,
            "not_matchable_comparison": 0,
            "is_identity_map": False,
        },
    }
