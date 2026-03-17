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
# Shared helpers for validation phases
# ---------------------------------------------------------------------------


def _load_map_for_validation(
    xodr_path: Path,
    osm_path: Path,
) -> tuple | None:
    """Load Lanelet2 map using geoReference from the XODR file.

    Handles preprocessed-OSM detection via the ``.mapping.json`` sidecar.

    Returns:
        ``(lanelet_map, offset_x, offset_y, conv_mapping, effective_osm)``
        or ``None`` if loading is not possible.
    """
    import json

    import lanelet2
    import lxml.etree as ET
    from autoware_lanelet2_extension_python.projection import MGRSProjector  # noqa: F401

    from .projection import latlon_to_lanelet2_origin
    from .road_lanelet_geo_mapping import (
        GeoRoadLaneletMapping,
        _cache_path_for,
        _preprocessed_osm_path_for,
    )

    mapping_path = _cache_path_for(xodr_path)
    if not mapping_path.exists():
        print(f"\n  Mapping file not found: {mapping_path}")
        print("  Skipping validation (no mapping file).")
        return None

    data = json.loads(mapping_path.read_text(encoding="utf-8"))
    conv_mapping = GeoRoadLaneletMapping.from_dict(data)

    effective_osm = osm_path
    if conv_mapping.preprocessing_log is not None:
        preprocessed_osm = _preprocessed_osm_path_for(xodr_path)
        if preprocessed_osm.exists():
            effective_osm = preprocessed_osm
            print(f"  Using preprocessed OSM: {preprocessed_osm}")
        else:
            print(
                f"  WARNING: Mapping has preprocessing_log but "
                f"{preprocessed_osm.name} not found.\n"
                f"  Validation would compare against a different lanelet set; skipping."
            )
            return None

    tree = ET.parse(str(xodr_path))
    geo_ref_elem = tree.find(".//geoReference")
    if geo_ref_elem is None or geo_ref_elem.text is None:
        print("  WARNING: No geoReference in XODR; skipping validation.")
        return None

    proj_string = geo_ref_elem.text.strip()
    lat_match = re.search(r"\+lat_0=([\d.eE+-]+)", proj_string)
    lon_match = re.search(r"\+lon_0=([\d.eE+-]+)", proj_string)
    if not lat_match or not lon_match:
        print("  WARNING: Cannot parse lat_0/lon_0 from geoReference; skipping.")
        return None

    origin_lat = float(lat_match.group(1))
    origin_lon = float(lon_match.group(1))

    origin = latlon_to_lanelet2_origin(origin_lat, origin_lon)
    projector = MGRSProjector(origin)
    lanelet_map = lanelet2.io.load(str(effective_osm), projector)

    fwd = projector.forward(lanelet2.core.GPSPoint(origin_lat, origin_lon, 0.0))
    offset_x = fwd.x
    offset_y = fwd.y

    return (lanelet_map, offset_x, offset_y, conv_mapping, effective_osm)


# ---------------------------------------------------------------------------
# Mapping cross-validation (standalone, for CLI use without convert context)
# ---------------------------------------------------------------------------


def run_mapping_validation(xodr_path: Path, osm_path: Path) -> bool:
    """Run mapping cross-validation between XODR and OSM files.

    Loads the Lanelet2 map from *osm_path*, parses the XODR to extract road
    reference lines and lane IDs, and checks that the ``.mapping.json`` next
    to the XODR is consistent with the geometric boundary comparison.

    Args:
        xodr_path: Path to the OpenDRIVE file.
        osm_path: Path to the Lanelet2 OSM file.

    Returns:
        ``True`` if mapping validation passed or was skipped (no mapping file),
        ``False`` if a mismatch was detected.
    """
    from .road_lanelet_geo_mapping import (
        MappingMismatchError,
        build_mapping,
        parse_roads_from_xodr,
        validate_mapping_consistency,
    )

    result = _load_map_for_validation(xodr_path, osm_path)
    if result is None:
        return True

    lanelet_map, offset_x, offset_y, conv_mapping, _effective_osm = result

    print(f"\n  Mapping file : {xodr_path.parent / (xodr_path.stem + '.mapping.json')}")
    print(f"  Entries      : {len(conv_mapping.lanelet_to_road_and_lane)}")

    roads = parse_roads_from_xodr(xodr_path)

    geo_mapping = build_mapping(
        lanelet_map,
        roads,
        (offset_x, offset_y),
        conv_mapping.xodr_sha256,
        conv_mapping.osm_sha256,
    )

    try:
        validate_mapping_consistency(conv_mapping.lanelet_to_road_and_lane, geo_mapping)
        print("  Mapping cross-validation: PASSED")
        return True
    except MappingMismatchError as e:
        print(f"  Mapping cross-validation: FAILED\n  {e}")
        return False


# ---------------------------------------------------------------------------
# Stop line XODR parsing
# ---------------------------------------------------------------------------

# Signal name patterns for stop line related signals
_STOP_LINE_SIGNAL_RE = re.compile(r"^(StopLine|StopSign|YieldSign)_(\d+)$")


def _parse_stop_lines_from_xodr(
    xodr_path: Path,
) -> tuple[dict[int, int], dict[int, list[int]], set[int]]:
    """Parse stop line objects, signals, and all signal IDs from an XODR file.

    Args:
        xodr_path: Path to the OpenDRIVE file.

    Returns:
        Tuple of:
        - ``object_to_road``: Mapping from stop line object ID to road ID.
        - ``linestring_to_signal_types``: Mapping from linestring ID (extracted
          from signal name) to list of signal type integers.
        - ``all_signal_ids``: Set of all signal IDs in the XODR.
    """
    import lxml.etree as ET

    tree = ET.parse(str(xodr_path))
    root = tree.getroot()

    object_to_road: dict[int, int] = {}
    linestring_to_signal_types: dict[int, list[int]] = {}
    all_signal_ids: set[int] = set()

    for road_elem in root.iter("road"):
        road_id = int(road_elem.get("id", "-1"))

        # Parse stop line objects
        for obj_elem in road_elem.iter("object"):
            obj_type = obj_elem.get("type", "")
            obj_name = obj_elem.get("name", "")
            obj_id_str = obj_elem.get("id", "")
            if not obj_id_str:
                continue

            # Standard format: type="stopLine"
            # CARLA format: type="-1" and name="Stencil_STOP"
            is_stop_line = obj_type == "stopLine" or (
                obj_type == "-1" and obj_name == "Stencil_STOP"
            )
            if is_stop_line:
                object_to_road[int(obj_id_str)] = road_id

        # Parse signals
        for signal_elem in road_elem.iter("signal"):
            sig_id_str = signal_elem.get("id", "")
            if sig_id_str:
                all_signal_ids.add(int(sig_id_str))

            sig_name = signal_elem.get("name", "")
            sig_type_str = signal_elem.get("type", "")
            match = _STOP_LINE_SIGNAL_RE.match(sig_name)
            if match and sig_type_str:
                linestring_id = int(match.group(2))
                signal_type = int(sig_type_str)
                linestring_to_signal_types.setdefault(linestring_id, []).append(
                    signal_type
                )

    return object_to_road, linestring_to_signal_types, all_signal_ids


def _parse_signal_dependencies_from_xodr(
    xodr_path: Path,
) -> dict[int, list[int]]:
    """Parse dependency references from stop line signals in the XODR.

    Returns:
        Mapping from signal ID to list of dependency IDs.
    """
    import lxml.etree as ET

    tree = ET.parse(str(xodr_path))
    root = tree.getroot()

    signal_dependencies: dict[int, list[int]] = {}
    for signal_elem in root.iter("signal"):
        sig_name = signal_elem.get("name", "")
        sig_id_str = signal_elem.get("id", "")
        if not _STOP_LINE_SIGNAL_RE.match(sig_name) or not sig_id_str:
            continue

        sig_id = int(sig_id_str)
        deps: list[int] = []
        for dep_elem in signal_elem.findall("dependency"):
            dep_id_str = dep_elem.get("id", "")
            if dep_id_str:
                deps.append(int(dep_id_str))
        if deps:
            signal_dependencies[sig_id] = deps

    return signal_dependencies


# ---------------------------------------------------------------------------
# Stop line validation
# ---------------------------------------------------------------------------


def run_stop_line_validation(xodr_path: Path, osm_path: Path) -> bool:
    """Run stop line cross-validation between XODR, OSM, and .mapping.json.

    Checks:
    1. XODR -> LL2 existence: stop line objects in XODR must exist in LL2 (FAIL if not)
    2. LL2 -> XODR existence: LL2 stop lines not in XODR are PASS (with logged reason)
    3. Cross-validation: road_id and signal_types must match between mapping and XODR
    4. Dependency integrity: dependency IDs must reference existing signals

    Args:
        xodr_path: Path to the OpenDRIVE file.
        osm_path: Path to the Lanelet2 OSM file.

    Returns:
        ``True`` if validation passed, ``False`` if errors were found.
    """
    result = _load_map_for_validation(xodr_path, osm_path)
    if result is None:
        print("  Skipping stop line validation (no mapping data).")
        return True

    lanelet_map, _offset_x, _offset_y, conv_mapping, _effective_osm = result

    # Check if mapping has stop line data (version 4)
    if conv_mapping.stop_line_mapping is None:
        print("  Mapping file has no stop_line_mapping (version < 4); skipping.")
        return True

    # Parse XODR stop line information
    xodr_objects, xodr_signal_types, all_signal_ids = _parse_stop_lines_from_xodr(
        xodr_path
    )
    signal_deps = _parse_signal_dependencies_from_xodr(xodr_path)

    # Get LL2 stop line IDs
    ll2_stop_line_ids: set[int] = set()
    for ls in lanelet_map.lineStringLayer:
        if "type" in ls.attributes and ls.attributes["type"] == "stop_line":
            ll2_stop_line_ids.add(ls.id)

    errors: list[str] = []
    warnings: list[str] = []

    # ── Check 1: XODR → LL2 existence (FAIL if XODR has stop lines not in LL2)
    xodr_only_objects: list[int] = []
    for obj_id in xodr_objects:
        if obj_id not in ll2_stop_line_ids:
            xodr_only_objects.append(obj_id)
    if xodr_only_objects:
        for obj_id in xodr_only_objects:
            errors.append(
                f"  XODR object {obj_id} (road {xodr_objects[obj_id]}): "
                f"not found in LL2 stop lines"
            )

    # ── Check 2: LL2 → XODR existence (PASS, log reason)
    ll2_not_in_xodr: list[int] = []
    for ls_id in sorted(ll2_stop_line_ids):
        if ls_id not in xodr_objects:
            ll2_not_in_xodr.append(ls_id)
            # Check if we have a skip reason from conversion
            if (
                conv_mapping.skipped_stop_lines
                and ls_id in conv_mapping.skipped_stop_lines
            ):
                reason = conv_mapping.skipped_stop_lines[ls_id].reason
                print(f"  linestring {ls_id}: skipped (reason: {reason})")
            elif ls_id in conv_mapping.stop_line_mapping:
                # In mapping but not in XODR objects -> unexpected
                warnings.append(
                    f"  linestring {ls_id}: in mapping but missing from XODR objects"
                )
            else:
                warnings.append(
                    f"  linestring {ls_id}: not in mapping or skipped_stop_lines "
                    f"(possible recording gap)"
                )

    # ── Check 3: Cross-validation (mapping vs XODR)
    cross_ok = 0
    cross_fail = 0
    for ls_id, entry in conv_mapping.stop_line_mapping.items():
        # Check road_id
        xodr_road = xodr_objects.get(ls_id)
        if xodr_road is None:
            # Object not in XODR — already covered by check 2 warnings
            continue
        if xodr_road != entry.road_id:
            errors.append(
                f"  linestring {ls_id}: road_id mismatch — "
                f"mapping={entry.road_id}, XODR={xodr_road}"
            )
            cross_fail += 1
            continue

        # Check signal_types
        xodr_types = sorted(xodr_signal_types.get(ls_id, []))
        mapping_types = sorted(entry.signal_types)
        if xodr_types != mapping_types:
            errors.append(
                f"  linestring {ls_id}: signal_types mismatch — "
                f"mapping={mapping_types}, XODR={xodr_types}"
            )
            cross_fail += 1
        else:
            cross_ok += 1

    # ── Check 4: Dependency integrity
    dep_errors = 0
    for sig_id, dep_ids in signal_deps.items():
        for dep_id in dep_ids:
            if dep_id not in all_signal_ids:
                errors.append(
                    f"  signal {sig_id}: dependency {dep_id} not found "
                    f"in XODR signal IDs"
                )
                dep_errors += 1

    # ── Summary
    print(f"\n  LL2 stop lines       : {len(ll2_stop_line_ids)}")
    print(f"  XODR stop line objects: {len(xodr_objects)}")
    print(f"  LL2-only (skipped)   : {len(ll2_not_in_xodr)}")
    print(f"  XODR-only (invalid)  : {len(xodr_only_objects)}")
    print(f"  Cross-validation     : {cross_ok} ok, {cross_fail} mismatch")
    print(f"  Dependency integrity : {dep_errors} error(s)")

    if warnings:
        print(f"\n  Warnings ({len(warnings)}):")
        for w in warnings:
            print(w)

    if errors:
        print(f"\n  Errors ({len(errors)}):")
        for e in errors[:20]:
            print(e)
        if len(errors) > 20:
            print(f"  ... and {len(errors) - 20} more")
        print("  Stop line validation: FAILED")
        return False

    print("  Stop line validation: PASSED")
    return True


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

    # ── Stop line validation ──────────────────────────────────────────────
    print(f"\n{'=' * 72}")
    print("  Stop Line Validation")
    print(f"{'=' * 72}")

    stop_line_ok = run_stop_line_validation(xodr_path, osm_path)
    if not stop_line_ok:
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
