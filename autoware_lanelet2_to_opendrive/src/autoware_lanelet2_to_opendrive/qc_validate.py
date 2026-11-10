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

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.xodr_digest import build_digest

DEFAULT_IGNORE_FILE: Path = Path(__file__).parent / "conf" / "qc_ignore_patterns.txt"


def load_ignore_patterns(path: Path = DEFAULT_IGNORE_FILE) -> list[str]:
    """Load regex patterns from a text file.

    Strips blank and `#`-prefixed lines. Does NOT compile the patterns;
    caller decides when to compile.

    Returns an empty list if ``path`` does not exist so that importing
    callers (e.g. ``analyze_xodr.DEFAULT_IGNORE_PATTERNS``) do not fail
    at import time when the data file is absent from the install tree.
    """
    patterns: list[str] = []
    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return patterns
    for raw in text.splitlines():
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


def find_implausible_lane_widths(
    xodr_path: Path,
    threshold_m: float | None = None,
) -> list[str]:
    """Return one message per non-center lane whose sampled width is implausible.

    Neither the ``asam-qc-opendrive`` spec-compliance bundle this module
    otherwise runs, nor the topology checks in ``opendrive/validation.py``,
    inspect a lane's ``<width>`` values at all - an inflated lane (e.g. from
    a corrupted source coordinate) is legal OpenDRIVE and passes both
    silently. This reuses ``xodr_digest.build_digest`` (which samples every
    ``<width>`` polynomial segment via ``_lane_width_samples``/
    ``_width_stats`` - see those functions' docstrings) against the written
    ``.xodr`` file, rather than re-parsing it or re-implementing the
    sampling, so the CI-facing check and the ``xodr-digest`` tool always
    agree on what a lane's width is.

    This is the same heuristic threshold as
    ``opendrive.validation.check_lane_width_plausibility`` (used, warning
    only, during conversion in ``main.py``) - see
    ``config.LaneWidthValidationConstants`` for how ``threshold_m`` was
    calibrated and its caveats, in particular that it is tuned on a single
    reference map and cannot catch a shape change that leaves the sampled
    min, max, and mean all unchanged. Unlike the conversion-time check, a
    non-empty result here is meant to fail the build (see ``validate``
    below) - `qc-validate` is a CI gate, so unlike the converter itself it
    is expected to be re-run against a corrected map rather than silently
    tolerate a questionable one.

    Args:
        xodr_path: Path to an already-written ``.xodr`` file.
        threshold_m: Width (metres) above which a lane is reported. Defaults
            to ``DEFAULT_CONFIG.lane_width_validation.implausible_width_threshold_m``.

    Returns:
        One human-readable message per implausible (road, lane) pair, in
        ``digest["roads"]`` order (ascending road id). Empty if none exceed
        the threshold.
    """
    if threshold_m is None:
        threshold_m = DEFAULT_CONFIG.lane_width_validation.implausible_width_threshold_m

    digest = build_digest(xodr_path)
    messages: list[str] = []
    for road in digest["roads"]:
        for lane in road["lanes"]:
            if lane["width_max_m"] > threshold_m:
                messages.append(
                    f"Road {road['id']} Lane {lane['id']}: sampled width reaches "
                    f"{lane['width_max_m']:.2f}m (min {lane['width_min_m']:.2f}m), "
                    f"above the {threshold_m:.1f}m plausibility threshold"
                )
    return messages


def validate(
    xodr_path: Path,
    ignore_patterns: Iterable[str] | None = None,
) -> int:
    """Run the qc-framework OpenDRIVE bundle and return the ERROR count.

    The returned count also includes lane-width plausibility violations
    (``find_implausible_lane_widths``) - a failure mode the
    ``asam-qc-opendrive`` bundle itself cannot see, since an implausibly
    wide lane is still spec-legal OpenDRIVE. See that function's docstring
    for what it checks and its calibration caveats.
    """
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

        width_violations = find_implausible_lane_widths(xodr_path)
        for message in width_violations:
            print(f"qc-validate: implausible lane width: {message}", file=sys.stderr)
        errors += len(width_violations)

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
