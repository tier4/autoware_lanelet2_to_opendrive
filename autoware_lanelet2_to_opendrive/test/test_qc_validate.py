import sys
import tempfile
from unittest.mock import MagicMock

import pytest
from qc_baselib import IssueSeverity
from qc_baselib.models.result import StatusType

from autoware_lanelet2_to_opendrive import qc_validate
from autoware_lanelet2_to_opendrive.qc_validate import (
    DEFAULT_IGNORE_FILE,
    count_errors,
    find_implausible_lane_widths,
    load_ignore_patterns,
    main,
    validate,
)

# Minimal hand-written OpenDRIVE fixtures, matching the ``test_xodr_digest.py``
# convention: no real map data is needed to exercise ``<width>`` sampling.
_HEADER = """\
<header revMajor="1" revMinor="7" name="test-map" version="1.00"
        north="0.0" south="0.0" east="0.0" west="0.0"
        date="2026-06-16T23:29:43">
  <geoReference>+proj=utm +zone=54 +datum=WGS84</geoReference>
</header>
"""


def _xodr(body: str) -> str:
    return f"""<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
{_HEADER}
{body}
</OpenDRIVE>
"""


def _road(road_id: int, width_a: float) -> str:
    """A single-lane road whose lane carries one constant ``<width a=...>``."""
    return f"""
<road name="r{road_id}" length="10.0" id="{road_id}" junction="-1">
  <planView>
    <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0">
      <paramPoly3 aU="0" bU="1" cU="0" dU="0" aV="0" bV="0" cV="0" dV="0" pRange="arcLength"/>
    </geometry>
  </planView>
  <lanes>
    <laneSection s="0.0">
      <left>
        <lane id="1" type="driving" level="false">
          <width sOffset="0.0" a="{width_a}" b="0" c="0" d="0"/>
        </lane>
      </left>
      <center>
        <lane id="0" type="none" level="false"/>
      </center>
      <right/>
    </laneSection>
  </lanes>
</road>
"""


def _write_xodr(tmp_path, road_id: int, width_a: float, name: str = "test.xodr"):
    path = tmp_path / name
    path.write_text(_xodr(_road(road_id, width_a)), encoding="utf-8")
    return path


def test_load_ignore_patterns_from_file(tmp_path):
    p = tmp_path / "ignore.txt"
    p.write_text(
        "# comment line\n" "\n" "attribute 'rule'\n" "Element 'positionInertial'\n",
        encoding="utf-8",
    )
    patterns = load_ignore_patterns(p)
    assert patterns == ["attribute 'rule'", "Element 'positionInertial'"]


def test_default_ignore_file_exists_and_nonempty():
    assert DEFAULT_IGNORE_FILE.is_file()
    assert DEFAULT_IGNORE_FILE.read_text(encoding="utf-8").strip()


def _make_issue(level, description, loc_descriptions=()):
    issue = MagicMock()
    issue.level = level
    issue.description = description
    issue.locations = [MagicMock(description=d) for d in loc_descriptions]
    return issue


def _make_result(checker_issues_by_id):
    result = MagicMock()
    result.get_checker_ids.return_value = list(checker_issues_by_id.keys())

    def _get_checker(bundle, cid):
        checker = MagicMock()
        checker.status = StatusType.COMPLETED
        checker.issues = checker_issues_by_id[cid]
        return checker

    result.get_checker_result.side_effect = _get_checker
    return result


def test_count_errors_filters_ignored():
    result = _make_result(
        {
            "c1": [
                _make_issue(IssueSeverity.ERROR, "attribute 'rule' is not declared"),
                _make_issue(IssueSeverity.ERROR, "road length mismatch"),
                _make_issue(IssueSeverity.WARNING, "pretty small warning"),
            ],
        }
    )
    assert count_errors(result, ["attribute 'rule'"]) == 1


def test_count_errors_skips_skipped_checkers():
    result = _make_result(
        {"c_skipped": [_make_issue(IssueSeverity.ERROR, "would not count")]}
    )

    # Override the mock: force SKIPPED on c_skipped
    def _skipped_checker(bundle, cid):
        checker = MagicMock()
        checker.status = StatusType.SKIPPED
        checker.issues = [_make_issue(IssueSeverity.ERROR, "would not count")]
        return checker

    result.get_checker_result.side_effect = _skipped_checker
    assert count_errors(result, []) == 0


def test_validate_cleans_up_temp_file_on_exception(monkeypatch, tmp_path):
    """validate() must delete its .xqar temp file even when run_checks raises."""

    def _boom(*_args, **_kwargs):
        raise RuntimeError("run_checks exploded")

    monkeypatch.setattr(qc_validate, "run_checks", _boom)
    # Redirect tempfile to ``tmp_path`` so the .xqar lands somewhere we
    # control and check, regardless of platform tempdir conventions
    # (e.g. ``/var/folders/...`` on macOS vs ``/tmp`` on Linux).
    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path))

    before = set(tmp_path.glob("tmp*.xqar"))
    with pytest.raises(RuntimeError, match="run_checks exploded"):
        validate(tmp_path / "does_not_need_to_exist.xodr", ignore_patterns=[])
    after = set(tmp_path.glob("tmp*.xqar"))

    leaked = after - before
    assert leaked == set(), f"validate() leaked temp files: {leaked}"


def test_main_prints_fail_and_exits_2_on_exception(monkeypatch, capsys, tmp_path):
    """main() must surface unexpected validate() exceptions with exit code 2."""

    def _boom(*_args, **_kwargs):
        raise RuntimeError("boom")

    monkeypatch.setattr(qc_validate, "validate", _boom)

    fake_xodr = tmp_path / "fake.xodr"
    fake_xodr.write_text("<OpenDRIVE/>", encoding="utf-8")
    monkeypatch.setattr(sys, "argv", ["qc_validate", str(fake_xodr)])

    with pytest.raises(SystemExit) as excinfo:
        main()

    assert excinfo.value.code == 2
    captured = capsys.readouterr()
    assert "qc-validate: FAIL (exception:" in captured.err
    assert "RuntimeError: boom" in captured.err


def test_find_implausible_lane_widths_no_violation_on_healthy_input(tmp_path):
    """A lane width within the calibrated distribution reports nothing."""
    xodr_path = _write_xodr(tmp_path, road_id=1, width_a=3.5)

    assert find_implausible_lane_widths(xodr_path) == []


def test_find_implausible_lane_widths_detects_violation(tmp_path):
    """A lane sampled above the default threshold is reported once."""
    xodr_path = _write_xodr(tmp_path, road_id=1, width_a=22.0)

    messages = find_implausible_lane_widths(xodr_path)

    assert len(messages) == 1
    assert "Road 1 Lane 1" in messages[0]
    assert "22.00m" in messages[0]


def test_find_implausible_lane_widths_respects_custom_threshold(tmp_path):
    """An explicit ``threshold_m`` overrides the config default."""
    xodr_path = _write_xodr(tmp_path, road_id=1, width_a=5.0)

    assert find_implausible_lane_widths(xodr_path, threshold_m=10.0) == []
    assert len(find_implausible_lane_widths(xodr_path, threshold_m=4.0)) == 1


def test_validate_fails_on_implausible_lane_width(monkeypatch, capsys, tmp_path):
    """``validate()`` folds width violations into its returned error count.

    ``run_checks`` is stubbed to a no-op (mirrors
    ``test_validate_cleans_up_temp_file_on_exception``'s approach) so this
    exercises only the width-plausibility path added on top of the
    ``asam-qc-opendrive`` bundle, without depending on that bundle's own
    checks passing against a deliberately minimal fixture.
    """

    def _noop_run_checks(*_args, **_kwargs):
        return None

    monkeypatch.setattr(qc_validate, "run_checks", _noop_run_checks)

    xodr_path = _write_xodr(tmp_path, road_id=1, width_a=22.0)

    errors = validate(xodr_path, ignore_patterns=[])

    assert errors == 1
    captured = capsys.readouterr()
    assert "qc-validate: implausible lane width" in captured.err
    assert "Road 1 Lane 1" in captured.err


def test_validate_passes_on_healthy_lane_width(monkeypatch, capsys, tmp_path):
    """``validate()`` reports zero errors when no lane exceeds the threshold."""

    def _noop_run_checks(*_args, **_kwargs):
        return None

    monkeypatch.setattr(qc_validate, "run_checks", _noop_run_checks)

    xodr_path = _write_xodr(tmp_path, road_id=1, width_a=3.5)

    errors = validate(xodr_path, ignore_patterns=[])

    assert errors == 0
    captured = capsys.readouterr()
    assert "implausible lane width" not in captured.err
