"""Slim qc-framework validation entrypoint.

Wraps asam-qc-opendrive for spec-compliance validation with a commit-tracked
ignore-list. Intended for CI as a gate on converter output. For full mapping
cross-validation and the longer analysis report, use `analyze_xodr.py`.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import tempfile
from pathlib import Path
from typing import Iterable

from qc_baselib import Configuration, IssueSeverity, Result
from qc_baselib.models.result import StatusType
from qc_opendrive import constants
from qc_opendrive.main import run_checks

DEFAULT_IGNORE_FILE: Path = Path(__file__).parent / "conf" / "qc_ignore_patterns.txt"


def load_ignore_patterns(path: Path = DEFAULT_IGNORE_FILE) -> list[str]:
    """Load regex patterns from a text file.

    Strips blank and `#`-prefixed lines. Does NOT compile the patterns;
    caller decides when to compile.
    """
    patterns: list[str] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        patterns.append(line)
    return patterns


def count_errors(
    result: Result,
    ignore_patterns: Iterable[str] = (),
) -> int:
    """Return the number of ERROR-level issues after filtering."""
    compiled = [re.compile(p) for p in ignore_patterns]

    def is_ignored(issue) -> bool:  # type: ignore[no-untyped-def]
        texts = [issue.description] + [
            loc.description for loc in issue.locations if loc.description
        ]
        return any(c.search(t) for c in compiled for t in texts)

    errors = 0
    for cid in result.get_checker_ids(constants.BUNDLE_NAME):
        checker = result.get_checker_result(constants.BUNDLE_NAME, cid)
        if checker.status == StatusType.SKIPPED:
            continue
        for issue in checker.issues:
            if issue.level != IssueSeverity.ERROR:
                continue
            if is_ignored(issue):
                continue
            errors += 1
    return errors


def validate(
    xodr_path: Path,
    ignore_patterns: Iterable[str] | None = None,
) -> int:
    """Run the qc-framework OpenDRIVE bundle and return the ERROR count."""
    patterns = (
        list(ignore_patterns) if ignore_patterns is not None else load_ignore_patterns()
    )
    with tempfile.NamedTemporaryFile(suffix=".xqar", delete=False) as tmp:
        result_path = tmp.name
    try:
        config = Configuration()
        config.set_config_param("InputFile", str(xodr_path))
        config.register_checker_bundle(constants.BUNDLE_NAME)
        config.set_checker_bundle_param(
            constants.BUNDLE_NAME, "resultFile", result_path
        )

        # qc-opendrive's run_checks populates a Result object in-place; the second
        # argument must be a Result (not a string path).  Mirrors analyze_xodr.py.
        result = Result()
        result.register_checker_bundle(
            name=constants.BUNDLE_NAME,
            description="OpenDrive checker bundle",
            version=constants.BUNDLE_VERSION,
            summary="",
        )
        result.set_result_version(version=constants.BUNDLE_VERSION)
        run_checks(config, result)

        errors = count_errors(result, patterns)
        return errors
    finally:
        try:
            os.unlink(result_path)
        except FileNotFoundError:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="qc-framework OpenDRIVE validator (CI gate).",
    )
    parser.add_argument("xodr", type=Path, help=".xodr file to validate")
    parser.add_argument(
        "--ignore-file",
        type=Path,
        default=DEFAULT_IGNORE_FILE,
        help="Path to ignore-patterns file (one regex per line).",
    )
    args = parser.parse_args()
    patterns = load_ignore_patterns(args.ignore_file)
    try:
        errors = validate(args.xodr, patterns)
    except Exception as exc:
        print(
            f"qc-validate: FAIL (exception: {type(exc).__name__}: {exc})",
            file=sys.stderr,
        )
        sys.exit(2)
    if errors > 0:
        print(f"qc-validate: FAIL ({errors} ERROR-level issues)", file=sys.stderr)
        sys.exit(1)
    print("qc-validate: PASS")
    sys.exit(0)
