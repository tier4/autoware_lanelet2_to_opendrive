"""
Analyze an OpenDRIVE (.xodr) file using ASAM QC OpenDRIVE checker.

Runs all available checks (basic, schema, semantic, geometry, performance,
smoothness) against a given .xodr file and prints a human-readable report.

Usage:
    uv run analyze <path_to_xodr_file> [options]

Examples:
    uv run analyze test/data/nishishinjuku.xodr
    uv run analyze output.xodr --output result.xqar
    uv run analyze output.xodr --min-severity WARNING
    uv run analyze output.xodr --max-issues 20
    uv run analyze output.xodr --ignore-pattern "attribute 'rule'"
    uv run analyze output.xodr --ignore-pattern "attribute 'rule'" --ignore-pattern "other pattern"
"""

import argparse
import logging
import re
import sys
import tempfile
from pathlib import Path

from qc_baselib import Configuration, IssueSeverity, Result
from qc_opendrive import constants
from qc_opendrive.main import run_checks

# Silence verbose library logging (overridden by --verbose)
logging.getLogger().setLevel(logging.WARNING)

SEVERITY_LABEL = {
    IssueSeverity.ERROR: "ERROR",
    IssueSeverity.WARNING: "WARNING",
    IssueSeverity.INFORMATION: "INFO",
}

# Lower order = more severe
SEVERITY_ORDER = {
    IssueSeverity.ERROR: 0,
    IssueSeverity.WARNING: 1,
    IssueSeverity.INFORMATION: 2,
}

MIN_SEVERITY_MAP = {
    "ERROR": IssueSeverity.ERROR,
    "WARNING": IssueSeverity.WARNING,
    "INFO": IssueSeverity.INFORMATION,
    "INFORMATION": IssueSeverity.INFORMATION,
}


def build_config(xodr_path: str, result_file: str) -> Configuration:
    """Build a Configuration object programmatically from a .xodr file path."""
    config = Configuration()
    config.set_config_param("InputFile", xodr_path)
    config.register_checker_bundle(constants.BUNDLE_NAME)
    config.set_checker_bundle_param(constants.BUNDLE_NAME, "resultFile", result_file)
    return config


def format_location(location) -> str:  # type: ignore[no-untyped-def]
    """Format a LocationType into a readable string."""
    parts = []
    if location.description:
        parts.append(f"[{location.description}]")
    for xml_loc in location.xml_location:
        parts.append(f"xpath={xml_loc.xpath}")
    for file_loc in location.file_location:
        parts.append(f"line={file_loc.row}, col={file_loc.column}")
    for inertial in location.inertial_location:
        parts.append(f"x={inertial.x:.3f}, y={inertial.y:.3f}, z={inertial.z:.3f}")
    return "  ".join(parts) if parts else "(no location)"


def print_report(
    result: Result,
    xodr_path: str,
    min_severity: IssueSeverity = IssueSeverity.INFORMATION,
    max_issues_per_checker: int = 10,
    ignore_patterns: list[str] | None = None,
) -> int:
    """
    Print a human-readable analysis report.

    Args:
        result: The QC result object.
        xodr_path: Path to the analyzed .xodr file.
        min_severity: Minimum severity level to display.
        max_issues_per_checker: Maximum issues to show per checker.
        ignore_patterns: List of regex patterns. Issues whose description
            matches any pattern are excluded from the report and error count.

    Returns the total number of ERROR-level issues (after filtering).
    """
    from qc_baselib.models.result import StatusType

    compiled_patterns = [re.compile(p) for p in (ignore_patterns or [])]

    def is_ignored(issue) -> bool:  # type: ignore[no-untyped-def]
        # Match against issue description and all location descriptions
        texts = [issue.description] + [
            loc.description for loc in issue.locations if loc.description
        ]
        return any(p.search(t) for p in compiled_patterns for t in texts)

    bundle_name = constants.BUNDLE_NAME
    checker_ids = result.get_checker_ids(bundle_name)

    # Aggregate issues across all checkers
    issues_by_severity: dict[IssueSeverity, list] = {
        IssueSeverity.ERROR: [],
        IssueSeverity.WARNING: [],
        IssueSeverity.INFORMATION: [],
    }
    skipped_checkers: list[tuple[str, str]] = []
    error_checkers: list[tuple[str, str]] = []
    ignored_count = 0

    for checker_id in checker_ids:
        checker = result.get_checker_result(bundle_name, checker_id)
        if checker.status == StatusType.SKIPPED:
            skipped_checkers.append((checker_id, checker.summary))
            continue
        if checker.status == StatusType.ERROR:
            error_checkers.append((checker_id, checker.summary))
        for issue in checker.issues:
            if is_ignored(issue):
                ignored_count += 1
                continue
            issues_by_severity[issue.level].append((checker_id, issue))

    total_errors = len(issues_by_severity[IssueSeverity.ERROR])
    total_warnings = len(issues_by_severity[IssueSeverity.WARNING])
    total_info = len(issues_by_severity[IssueSeverity.INFORMATION])
    total_all = total_errors + total_warnings + total_info

    min_order = SEVERITY_ORDER[min_severity]

    # ── Header ──────────────────────────────────────────────────────────────
    print("=" * 72)
    print("  ASAM QC OpenDRIVE Analysis Report")
    print(f"  File   : {xodr_path}")
    print(f"  Bundle : {bundle_name}  (version {constants.BUNDLE_VERSION})")
    print("=" * 72)

    # ── Checker execution summary ───────────────────────────────────────────
    total_checkers = len(checker_ids)
    completed = total_checkers - len(skipped_checkers) - len(error_checkers)
    print(
        f"\nCheckers : {total_checkers} total  |  "
        f"{completed} completed  |  "
        f"{len(skipped_checkers)} skipped  |  "
        f"{len(error_checkers)} errored"
    )

    if error_checkers:
        print("\n[!] Checkers that encountered errors:")
        for cid, summary in error_checkers:
            print(f"    - {cid}")
            if summary:
                print(f"      {summary}")

    if skipped_checkers:
        print("\n[-] Skipped checkers " "(preconditions not met or version mismatch):")
        for cid, summary in skipped_checkers:
            reason = summary.split(".")[0] if summary else ""
            print(f"    - {cid}")
            if reason:
                print(f"      Reason: {reason}")

    # ── Issue summary ───────────────────────────────────────────────────────
    print(f"\n{'─' * 72}")
    ignored_suffix = f"  |  {ignored_count} ignored" if ignored_count > 0 else ""
    print(
        f"Issues   : {total_all} total  |  "
        f"{total_errors} errors  |  "
        f"{total_warnings} warnings  |  "
        f"{total_info} info"
        f"{ignored_suffix}"
    )
    print(f"{'─' * 72}")

    if total_all == 0:
        print("\nNo issues found.")

    # ── Detailed issues (grouped by severity then checker) ──────────────────
    for severity in (
        IssueSeverity.ERROR,
        IssueSeverity.WARNING,
        IssueSeverity.INFORMATION,
    ):
        # Skip severities below the requested minimum (higher order = less severe)
        if SEVERITY_ORDER[severity] > min_order:
            continue
        issues = issues_by_severity[severity]
        if not issues:
            continue

        label = SEVERITY_LABEL[severity]
        print(f"\n[{label}] ({len(issues)} issue{'s' if len(issues) != 1 else ''})")
        print("─" * 72)

        # Group by checker
        by_checker: dict[str, list] = {}
        for checker_id, issue in issues:
            by_checker.setdefault(checker_id, []).append(issue)

        for checker_id, checker_issues in by_checker.items():
            checker = result.get_checker_result(bundle_name, checker_id)
            print(f"\n  Checker : {checker_id}")
            if checker.description:
                print(f"  Rule    : {checker.description}")
            if checker_issues[0].rule_uid:
                print(f"  RuleUID : {checker_issues[0].rule_uid}")
            total_for_checker = len(checker_issues)
            shown = min(total_for_checker, max_issues_per_checker)
            print(
                f"  Issues  : {total_for_checker}"
                + (
                    f"  (showing first {shown})"
                    if total_for_checker > max_issues_per_checker
                    else ""
                )
            )

            for issue in checker_issues[:max_issues_per_checker]:
                print(f"\n    #{issue.issue_id:>5}  {issue.description}")
                for loc in issue.locations:
                    print(f"             {format_location(loc)}")

            if total_for_checker > max_issues_per_checker:
                remaining = total_for_checker - max_issues_per_checker
                print(
                    f"\n    ... and {remaining} more issue(s). "
                    "Use --max-issues to show more."
                )

    # ── Footer ──────────────────────────────────────────────────────────────
    print(f"\n{'=' * 72}")
    if total_errors > 0:
        print(f"  RESULT: FAIL  ({total_errors} error(s) found)")
    elif total_warnings > 0:
        print(f"  RESULT: PASS with warnings  ({total_warnings} warning(s))")
    else:
        print("  RESULT: PASS  (no errors or warnings found)")
    print("=" * 72)

    return total_errors


def main() -> None:
    """Entry point for the analyze command."""
    parser = argparse.ArgumentParser(
        prog="analyze",
        description="Analyze a .xodr file with ASAM QC OpenDRIVE checker.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("xodr_file", help="Path to the .xodr file to analyze")
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help=(
            "Path for the .xqar result file "
            "(default: temp file, deleted after report)"
        ),
    )
    parser.add_argument(
        "--min-severity",
        default="INFO",
        choices=["ERROR", "WARNING", "INFO", "INFORMATION"],
        help="Minimum severity level to display (default: INFO)",
    )
    parser.add_argument(
        "--max-issues",
        type=int,
        default=10,
        help="Maximum number of issues to display per checker (default: 10)",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose library logging",
    )
    parser.add_argument(
        "--fail-on-warning",
        action="store_true",
        help="Exit with non-zero code if any warnings are found",
    )
    parser.add_argument(
        "--ignore-pattern",
        action="append",
        default=[],
        metavar="PATTERN",
        dest="ignore_patterns",
        help=(
            "Regex pattern to ignore matching issues (can be specified multiple times). "
            "Matched issues are excluded from the report and do not affect the exit code. "
            "Example: --ignore-pattern \"attribute 'rule'\""
        ),
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.INFO)

    xodr_path = Path(args.xodr_file).resolve()
    if not xodr_path.exists():
        print(f"Error: file not found: {xodr_path}", file=sys.stderr)
        sys.exit(2)

    min_severity = MIN_SEVERITY_MAP[args.min_severity.upper()]

    # Determine result file path
    use_temp = args.output is None
    if use_temp:
        tmp = tempfile.NamedTemporaryFile(suffix=".xqar", delete=False)
        result_file = tmp.name
        tmp.close()
    else:
        result_file = str(Path(args.output).resolve())

    # Build config and run all checks
    config = build_config(str(xodr_path), result_file)

    result_obj = Result()
    result_obj.register_checker_bundle(
        name=constants.BUNDLE_NAME,
        description="OpenDrive checker bundle",
        version=constants.BUNDLE_VERSION,
        summary="",
    )
    result_obj.set_result_version(version=constants.BUNDLE_VERSION)

    run_checks(config, result_obj)

    # Print report BEFORE write_to_file so generate_summary=True does not
    # append "X issue(s) are found." to checker summaries before we read them.
    error_count = print_report(
        result_obj,
        str(xodr_path),
        min_severity,
        max_issues_per_checker=args.max_issues,
        ignore_patterns=args.ignore_patterns,
    )

    # Write result file (generate_summary appends issue counts to summaries)
    result_obj.copy_param_from_config(config)
    result_obj.write_to_file(result_file, generate_summary=True)

    if not use_temp:
        print(f"\nFull result written to: {result_file}")

    # Clean up temp file
    if use_temp:
        Path(result_file).unlink(missing_ok=True)

    # Exit code
    if error_count > 0:
        sys.exit(1)

    checker_ids = result_obj.get_checker_ids(constants.BUNDLE_NAME)
    warning_count = sum(
        sum(
            1
            for issue in result_obj.get_checker_result(
                constants.BUNDLE_NAME, cid
            ).issues
            if issue.level == IssueSeverity.WARNING
        )
        for cid in checker_ids
    )
    if args.fail_on_warning and warning_count > 0:
        sys.exit(1)

    sys.exit(0)


if __name__ == "__main__":
    main()
