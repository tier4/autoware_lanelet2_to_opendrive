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
    load_ignore_patterns,
    main,
    validate,
)


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
