"""Tests for ``test/data/tools/reduce_lanelet2_map.py``.

The reducer shrinks a Lanelet2 map to a small one on which a given check
still fails the same way. ``ddmin`` is the search; ``MapPredicate`` turns a
set of lanelet ids into a candidate map and asks the user's command about it.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import lxml.etree as ET
import pytest

_TOOLS = Path(__file__).parent / "data" / "tools"


def _load_tool():
    spec = importlib.util.spec_from_file_location(
        "reduce_lanelet2_map", _TOOLS / "reduce_lanelet2_map.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # ``@dataclass`` looks its defining module up in ``sys.modules``.
    sys.modules[spec.name] = module
    sys.path.insert(0, str(_TOOLS))  # it imports crop_lanelet2_map beside it
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(_TOOLS))
    return module


reduce_tool = _load_tool()


class _Counting:
    """A predicate that records every set it was asked about."""

    def __init__(self, interesting):
        self.interesting = interesting
        self.calls: list[frozenset[int]] = []

    def __call__(self, keep: frozenset[int]) -> bool:
        self.calls.append(keep)
        return self.interesting(keep)


# --- ddmin -------------------------------------------------------------------


def test_ddmin_finds_a_single_culprit() -> None:
    result = reduce_tool.ddmin(list(range(40)), _Counting(lambda s: 17 in s))
    assert result == [17]


def test_ddmin_keeps_every_element_that_is_needed_together() -> None:
    result = reduce_tool.ddmin(list(range(40)), _Counting(lambda s: {3, 21, 38} <= s))
    assert result == [3, 21, 38]


def test_ddmin_result_is_one_minimal() -> None:
    """Removing any single element of the result makes it uninteresting."""

    def interesting(s: frozenset[int]) -> bool:
        return (5 in s and 9 in s) or (30 in s and 31 in s and 32 in s)

    result = reduce_tool.ddmin(list(range(40)), _Counting(interesting))
    assert interesting(frozenset(result))
    for element in result:
        assert not interesting(frozenset(result) - {element})


def test_ddmin_never_asks_about_the_same_set_twice_or_the_empty_set() -> None:
    predicate = _Counting(lambda s: {3, 21} <= s)
    reduce_tool.ddmin(list(range(64)), predicate)
    assert len(predicate.calls) == len(set(predicate.calls))
    assert frozenset() not in predicate.calls


def test_ddmin_is_deterministic_and_sorted() -> None:
    items = [9, 2, 7, 4, 30, 11]
    first = reduce_tool.ddmin(items, _Counting(lambda s: {7, 30} <= s))
    second = reduce_tool.ddmin(list(reversed(items)), _Counting(lambda s: {7, 30} <= s))
    assert first == second == [7, 30]


def test_ddmin_refuses_a_start_that_is_not_interesting() -> None:
    with pytest.raises(reduce_tool.NotInteresting):
        reduce_tool.ddmin(list(range(10)), _Counting(lambda s: False))


# --- MapPredicate --------------------------------------------------------------


# Two lanelets, and strict XML: write_excerpt parses with lxml, which
# rejects walkway_mini.osm (a "--" inside its header comment).
SPEED_CHANGE = Path(__file__).parent / "data" / "speed_change_mini.osm"


def _lanelet_ids(osm: Path) -> list[int]:
    root = ET.parse(str(osm)).getroot()
    return sorted(
        int(r.get("id"))
        for r in root.iter("relation")
        if any(t.get("k") == "type" and t.get("v") == "lanelet" for t in r.iter("tag"))
    )


def test_map_predicate_writes_the_candidate_and_reads_the_exit_code(
    tmp_path: Path,
) -> None:
    ids = _lanelet_ids(SPEED_CHANGE)
    assert len(ids) >= 2, "fixture must hold at least two lanelets"
    wanted = ids[-1]
    # Interesting iff the candidate map still contains lanelet ``wanted``.
    command = [
        sys.executable,
        "-c",
        f"import sys; sys.exit(0 if 'id=\"{wanted}\"' in open(sys.argv[1]).read() else 1)",
        "{map}",
    ]
    predicate = reduce_tool.MapPredicate(SPEED_CHANGE, command, tmp_path)
    assert predicate(frozenset([wanted]))
    assert not predicate(frozenset(ids[:1]))


def test_reduce_end_to_end_on_a_small_map(tmp_path: Path) -> None:
    ids = _lanelet_ids(SPEED_CHANGE)
    wanted = ids[-1]
    command = [
        sys.executable,
        "-c",
        f"import sys; sys.exit(0 if 'id=\"{wanted}\"' in open(sys.argv[1]).read() else 1)",
        "{map}",
    ]
    out = tmp_path / "reduced.osm"
    result = reduce_tool.reduce_map(SPEED_CHANGE, out, command, work_dir=tmp_path / "w")
    assert result.kept == [wanted]
    assert _lanelet_ids(out) == [wanted]
    provenance = out.with_suffix(".provenance.json").read_text()
    assert str(wanted) in provenance
    assert "speed_change_mini.osm" in provenance
