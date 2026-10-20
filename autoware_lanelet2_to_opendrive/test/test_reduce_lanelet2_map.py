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


def _match_command(needle: str) -> list:
    """A command that exits 0 when the candidate contains *needle*."""
    return [
        sys.executable,
        "-c",
        f"import sys; sys.exit(0 if {needle!r} in open(sys.argv[1]).read() else 1)",
        "{map}",
    ]


def _lies_once_command(flag: Path) -> list:
    """Exits 0 on its first invocation and honestly (1) thereafter.

    The first call creates *flag*; later calls see it and refuse. Written as a
    subprocess rather than a Python predicate because `reduce_map` drives a
    command, and the defect is about what the command reports.
    """
    return [
        sys.executable,
        "-c",
        "import pathlib, sys; f = pathlib.Path(sys.argv[2]); "
        "sys.exit(1 if f.exists() else (f.write_text('x') and 0 or 0))",
        "{map}",
        str(flag),
    ]


class TestTheDeliveredFileIsRevalidated:
    """ddmin never backtracks and memoises, so one false positive is permanent.

    The search takes the first interesting chunk and caches every verdict, so
    a candidate judged interesting for the wrong reason -- a flaky command, or
    one that exits 0 on a map the converter could not even load -- is never
    re-asked. The result was then written with a confident provenance file
    beside it, and nothing had ever run the predicate on the delivered file.
    """

    def test_a_command_that_lies_once_yields_no_result(self, tmp_path: Path) -> None:
        destination = tmp_path / "reduced.osm"

        with pytest.raises(reduce_tool.NotInteresting) as refused:
            reduce_tool.reduce_map(
                SPEED_CHANGE,
                destination,
                _lies_once_command(tmp_path / "seen"),
                work_dir=tmp_path / "w",
            )

        assert "does not reproduce the failure" in str(refused.value)
        assert not destination.with_suffix(".provenance.json").exists(), (
            "provenance was written for a result that does not reproduce the "
            "failure, which is what makes a bad reduction look like a good one"
        )

    def test_provenance_records_the_start_set_and_the_confirmation(
        self, tmp_path: Path
    ) -> None:
        """ddmin's answer depends on the starting ids, so they have to be kept."""
        import json

        wanted = _lanelet_ids(SPEED_CHANGE)[-1]
        destination = tmp_path / "reduced.osm"

        reduce_tool.reduce_map(
            SPEED_CHANGE,
            destination,
            _match_command(f'id="{wanted}"'),
            work_dir=tmp_path / "w",
        )

        provenance = json.loads(
            destination.with_suffix(".provenance.json").read_text(encoding="utf-8")
        )
        assert provenance["start_lanelets"] == _lanelet_ids(SPEED_CHANGE), (
            "start_lanelets must be the ids, not their count: ddmin's result "
            "depends on the set and the order it is searched in"
        )
        assert provenance["result_reproduces_the_failure"] is True


class TestThePlaceholderWorksInsideAToken:
    """The Hydra form this tool's own docstring recommends is `key={map}`."""

    def test_a_key_value_argument_is_accepted(self, tmp_path: Path) -> None:
        predicate = reduce_tool.MapPredicate(
            SPEED_CHANGE, ["true", "input_map_path={map}"], tmp_path / "w"
        )
        assert predicate.command == ["true", "input_map_path={map}"]

    def test_a_command_without_the_placeholder_is_still_refused(
        self, tmp_path: Path
    ) -> None:
        with pytest.raises(ValueError, match=r"\{map\}"):
            reduce_tool.MapPredicate(SPEED_CHANGE, ["true"], tmp_path / "w")

    def test_the_substitution_reaches_inside_the_token(self, tmp_path: Path) -> None:
        """What the command receives has to be the candidate's real path."""
        work = tmp_path / "w"
        predicate = reduce_tool.MapPredicate(
            SPEED_CHANGE,
            [sys.executable, "-c", "import sys; sys.exit(0)", "input_map_path={map}"],
            work,
        )
        predicate(frozenset(_lanelet_ids(SPEED_CHANGE)))

        argv = (work / "eval-0001" / "argv.txt").read_text(encoding="utf-8")
        assert "input_map_path=" in argv
        assert "{map}" not in argv
        assert "candidate.osm" in argv


class TestEachEvaluationIsIsolated:
    """A candidate must not be judged on the previous candidate's output.

    The converter writes its `.xodr` and sidecars beside the input when given
    no output path, so a shared `candidate.osm` left them behind for the next
    evaluation to read.
    """

    def test_every_evaluation_gets_its_own_directory(self, tmp_path: Path) -> None:
        work = tmp_path / "w"
        predicate = reduce_tool.MapPredicate(
            SPEED_CHANGE,
            [sys.executable, "-c", "import sys; sys.exit(0)", "{map}"],
            work,
        )
        ids = _lanelet_ids(SPEED_CHANGE)

        predicate(frozenset(ids))
        predicate(frozenset(ids[:1]))

        for n in (1, 2):
            attempt = work / f"eval-{n:04d}"
            assert (attempt / "candidate.osm").is_file()
            assert (attempt / "stdout.log").is_file()
            assert (attempt / "kept.txt").is_file(), "the ids judged are not recorded"
