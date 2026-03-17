"""
Analyze an OpenDRIVE (.xodr) file using ASAM QC OpenDRIVE checker and
cross-validate the lanelet-to-road mapping against geometric comparison.

Runs all available checks (basic, schema, semantic, geometry, performance,
smoothness) against a given .xodr file, performs mapping cross-validation,
and prints a human-readable report.

Usage:
    uv run analyze <xodr_file> <osm_file> [options]

Examples:
    uv run analyze output.xodr input.osm
    uv run analyze output.xodr input.osm --output result.xqar
    uv run analyze output.xodr input.osm --min-severity WARNING
    uv run analyze output.xodr input.osm --max-issues 20
    uv run analyze output.xodr input.osm --ignore-pattern "attribute 'rule'"
    uv run analyze output.xodr input.osm --no-default-ignores
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

logger = logging.getLogger(__name__)

# Default ignore patterns for known false positives.
# The converter targets CARLA which uses OpenDRIVE 1.4 syntax but supports
# the rule attribute introduced in 1.7. The ASAM QC checker validates against
# the declared schema version (1.4), so rule on <road> triggers a false positive.
# Use --no-default-ignores to disable these defaults.
DEFAULT_IGNORE_PATTERNS: list[str] = [
    r"attribute 'rule'",  # <road rule="RHT/LHT"> valid in 1.7, false positive on 1.4
]

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


# ---------------------------------------------------------------------------
# Mapping cross-validation (standalone, for CLI use without convert context)
# ---------------------------------------------------------------------------


def run_mapping_validation(xodr_path: Path, osm_path: Path) -> bool:
    """Run mapping cross-validation between XODR and OSM files.

    Loads the Lanelet2 map from *osm_path*, parses the XODR with ``pyxodr``,
    and checks that the ``.mapping.json`` next to the XODR is consistent with
    the geometric boundary comparison.

    Args:
        xodr_path: Path to the OpenDRIVE file.
        osm_path: Path to the Lanelet2 OSM file.

    Returns:
        ``True`` if mapping validation passed or was skipped (no mapping file),
        ``False`` if a mismatch was detected.
    """
    import json

    import lanelet2
    import lxml.etree as ET
    from autoware_lanelet2_extension_python.projection import MGRSProjector  # noqa: F401
    from pyxodr.road_objects.network import RoadNetwork

    from .projection import latlon_to_lanelet2_origin
    from .road_lanelet_geo_mapping import (
        GeoRoadLaneletMapping,
        MappingMismatchError,
        _cache_path_for,
        _sha256_of_file,
        build_mapping,
        validate_mapping_consistency,
    )

    # Check for .mapping.json next to the XODR
    mapping_path = _cache_path_for(xodr_path)
    if not mapping_path.exists():
        print(f"\n  Mapping file not found: {mapping_path}")
        print("  Skipping mapping cross-validation.")
        return True

    data = json.loads(mapping_path.read_text(encoding="utf-8"))
    conv_mapping = GeoRoadLaneletMapping.from_dict(data)

    print(f"\n  Mapping file : {mapping_path}")
    print(f"  Entries      : {len(conv_mapping.lanelet_to_road_and_lane)}")

    # Parse geoReference from XODR to determine origin
    tree = ET.parse(str(xodr_path))
    geo_ref_elem = tree.find(".//geoReference")
    if geo_ref_elem is None or geo_ref_elem.text is None:
        print("  WARNING: No geoReference in XODR; skipping mapping validation.")
        return True

    proj_string = geo_ref_elem.text.strip()
    lat_match = re.search(r"\+lat_0=([\d.eE+-]+)", proj_string)
    lon_match = re.search(r"\+lon_0=([\d.eE+-]+)", proj_string)
    if not lat_match or not lon_match:
        print("  WARNING: Cannot parse lat_0/lon_0 from geoReference; skipping.")
        return True

    origin_lat = float(lat_match.group(1))
    origin_lon = float(lon_match.group(1))

    # Load Lanelet2 map with the same origin
    origin = latlon_to_lanelet2_origin(origin_lat, origin_lon)
    projector = MGRSProjector(origin)
    lanelet_map = lanelet2.io.load(str(osm_path), projector)

    # The geometric comparison subtracts mgrs_offset from lanelet coordinates.
    # In the standalone CLI context we cannot reliably reconstruct the meter
    # offset used during conversion, so we default to (0, 0).
    offset_x = 0.0
    offset_y = 0.0

    road_network = RoadNetwork.from_file(str(xodr_path))
    xodr_sha256 = _sha256_of_file(xodr_path)
    osm_sha256 = _sha256_of_file(osm_path)

    geo_mapping = build_mapping(
        lanelet_map, road_network, (offset_x, offset_y), xodr_sha256, osm_sha256
    )

    try:
        validate_mapping_consistency(conv_mapping.lanelet_to_road_and_lane, geo_mapping)
        print("  Mapping cross-validation: PASSED")
        return True
    except MappingMismatchError as e:
        print(f"  Mapping cross-validation: FAILED\n  {e}")
        return False


# ---------------------------------------------------------------------------
# Unified analysis entry-point (callable from convert and CLI)
# ---------------------------------------------------------------------------


def run_analysis(
    xodr_path: Path,
    osm_path: Path,
    *,
    min_severity: IssueSeverity = IssueSeverity.INFORMATION,
    max_issues_per_checker: int = 10,
    ignore_patterns: list[str] | None = None,
    fail_on_warning: bool = False,
    result_file: str | None = None,
) -> int:
    """Run ASAM QC checks and mapping cross-validation.

    This is the programmatic entry-point shared by the ``analyze`` CLI command
    and the ``convert`` post-conversion step.

    Args:
        xodr_path: Path to the .xodr file.
        osm_path: Path to the Lanelet2 .osm file.
        min_severity: Minimum severity level to report.
        max_issues_per_checker: Max issues shown per checker.
        ignore_patterns: Regex patterns to suppress matching issues.
        fail_on_warning: Treat warnings as failures.
        result_file: Path to persist the .xqar result (temp file if None).

    Returns:
        Number of errors found (ASAM QC errors + mapping mismatch).
    """
    xodr_path = xodr_path.resolve()
    osm_path = osm_path.resolve()

    if ignore_patterns is None:
        ignore_patterns = list(DEFAULT_IGNORE_PATTERNS)

    # ── ASAM QC checks ─────────────────────────────────────────────────────
    use_temp = result_file is None
    if use_temp:
        tmp = tempfile.NamedTemporaryFile(suffix=".xqar", delete=False)
        rf = tmp.name
        tmp.close()
    else:
        rf = str(Path(result_file).resolve())

    config = build_config(str(xodr_path), rf)

    result_obj = Result()
    result_obj.register_checker_bundle(
        name=constants.BUNDLE_NAME,
        description="OpenDrive checker bundle",
        version=constants.BUNDLE_VERSION,
        summary="",
    )
    result_obj.set_result_version(version=constants.BUNDLE_VERSION)

    run_checks(config, result_obj)

    error_count = print_report(
        result_obj,
        str(xodr_path),
        min_severity,
        max_issues_per_checker=max_issues_per_checker,
        ignore_patterns=ignore_patterns,
    )

    result_obj.copy_param_from_config(config)
    result_obj.write_to_file(rf, generate_summary=True)

    if not use_temp:
        print(f"\nFull result written to: {rf}")

    if use_temp:
        Path(rf).unlink(missing_ok=True)

    # ── Mapping cross-validation ───────────────────────────────────────────
    print(f"\n{'=' * 72}")
    print("  Mapping Cross-Validation")
    print(f"{'=' * 72}")

    mapping_ok = run_mapping_validation(xodr_path, osm_path)
    if not mapping_ok:
        error_count += 1

    # ── Final summary ──────────────────────────────────────────────────────
    print(f"\n{'=' * 72}")
    if error_count > 0:
        print(f"  FINAL RESULT: FAIL  ({error_count} error(s) found)")
    else:
        print("  FINAL RESULT: PASS")
    print(f"{'=' * 72}")

    # Check warnings for fail_on_warning
    if fail_on_warning and error_count == 0:
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
        if warning_count > 0:
            return 1

    return error_count


def main() -> None:
    """Entry point for the analyze command."""
    parser = argparse.ArgumentParser(
        prog="analyze",
        description=(
            "Analyze a .xodr file with ASAM QC OpenDRIVE checker "
            "and cross-validate the lanelet-to-road mapping."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("xodr_file", help="Path to the .xodr file to analyze")
    parser.add_argument("osm_file", help="Path to the Lanelet2 .osm file")
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
            "Added on top of the default ignore patterns. "
            "Example: --ignore-pattern \"attribute 'rule'\""
        ),
    )
    parser.add_argument(
        "--no-default-ignores",
        action="store_true",
        help=(
            f"Disable the built-in default ignore patterns: {DEFAULT_IGNORE_PATTERNS}. "
            "By default these known false positives are suppressed."
        ),
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.INFO)

    xodr_path = Path(args.xodr_file).resolve()
    if not xodr_path.exists():
        print(f"Error: file not found: {xodr_path}", file=sys.stderr)
        sys.exit(2)

    osm_path = Path(args.osm_file).resolve()
    if not osm_path.exists():
        print(f"Error: file not found: {osm_path}", file=sys.stderr)
        sys.exit(2)

    base_patterns = [] if args.no_default_ignores else DEFAULT_IGNORE_PATTERNS
    all_ignore_patterns = base_patterns + args.ignore_patterns

    error_count = run_analysis(
        xodr_path=xodr_path,
        osm_path=osm_path,
        min_severity=MIN_SEVERITY_MAP[args.min_severity.upper()],
        max_issues_per_checker=args.max_issues,
        ignore_patterns=all_ignore_patterns,
        fail_on_warning=args.fail_on_warning,
        result_file=args.output,
    )

    sys.exit(1 if error_count > 0 else 0)


if __name__ == "__main__":
    main()
