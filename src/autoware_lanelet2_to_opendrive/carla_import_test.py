"""
Validate a .xodr file against CARLA's map loading pipeline.

Runs two offline tests (no CARLA server required):
  1. OpenDRIVE Parser  – carla.Map() parse + waypoint/topology generation
  2. Traffic Manager   – cook_in_memory_map() binary cooking

An optional online test requires a running CARLA server:
  3. World Generation  – client.generate_opendrive_world()

Usage:
    uv run carla-import-test <path_to_xodr_file> [options]

Examples:
    uv run carla-import-test test/data/nishishinjuku.xodr
    uv run carla-import-test map.xodr --skip-tm
    uv run carla-import-test map.xodr --host localhost --port 2000
    uv run carla-import-test map.xodr --map-name MyMap --verbose
"""

from __future__ import annotations

import argparse
import logging
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

logger = logging.getLogger(__name__)

# Silence verbose library logging (overridden by --verbose)
logging.getLogger().setLevel(logging.WARNING)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CarlaImportConstants:
    """Constants for CARLA import validation tests.

    Attributes:
        waypoint_generation_distance: Distance interval in metres for waypoint generation
        default_carla_port: Default TCP port for CARLA server connections
        server_connection_timeout: Timeout in seconds for CARLA server connection
    """

    waypoint_generation_distance: float = 1.0
    default_carla_port: int = 2000
    server_connection_timeout: float = 10.0


CONSTANTS = CarlaImportConstants()


# ---------------------------------------------------------------------------
# Result / Report dataclasses
# ---------------------------------------------------------------------------


@dataclass
class TestResult:
    """Result of a single CARLA import validation test.

    Attributes:
        name: Human-readable test name
        passed: Whether the test passed
        message: One-line summary message
        detail: Optional additional detail (e.g., counts, file paths)
    """

    name: str
    passed: bool
    message: str
    detail: str = ""

    def __bool__(self) -> bool:
        return self.passed


@dataclass
class ImportTestReport:
    """Aggregated report for all CARLA import tests.

    Attributes:
        xodr_path: Path to the .xodr file under test
        results: Ordered list of TestResult objects
    """

    xodr_path: str
    results: list[TestResult] = field(default_factory=list)

    @property
    def all_passed(self) -> bool:
        """True if every test in the report passed."""
        return bool(self.results) and all(r.passed for r in self.results)

    @property
    def passed_count(self) -> int:
        """Number of tests that passed."""
        return sum(1 for r in self.results if r.passed)

    @property
    def failed_count(self) -> int:
        """Number of tests that failed."""
        return sum(1 for r in self.results if not r.passed)


# ---------------------------------------------------------------------------
# Test functions
# ---------------------------------------------------------------------------


def run_opendrive_parser_test(xodr_content: str, map_name: str) -> TestResult:
    """Test CARLA's OpenDRIVE parser with the provided map content.

    Parses the .xodr content using carla.Map() (offline, no server required),
    then verifies the road graph by generating waypoints and extracting topology.

    Args:
        xodr_content: Raw XML content of the .xodr file
        map_name: Name to assign to the map inside CARLA

    Returns:
        TestResult indicating pass/fail with waypoint and topology counts
    """
    test_name = "OpenDRIVE Parser Test"
    try:
        import carla  # type: ignore[import]

        carla_map = carla.Map(map_name, xodr_content)
        waypoints = carla_map.generate_waypoints(CONSTANTS.waypoint_generation_distance)
        topology = carla_map.get_topology()
        waypoint_count = len(waypoints)
        topology_count = len(topology)
        detail = (
            f"{waypoint_count} waypoints generated, " f"{topology_count} topology edges"
        )
        logger.debug("Parser test detail: %s", detail)
        return TestResult(
            name=test_name,
            passed=True,
            message="Parsed successfully",
            detail=detail,
        )
    except RuntimeError as exc:
        return TestResult(
            name=test_name,
            passed=False,
            message=f"CARLA RuntimeError: {exc}",
        )
    except ImportError as exc:
        return TestResult(
            name=test_name,
            passed=False,
            message=f"carla package not available: {exc}",
        )
    except Exception as exc:  # noqa: BLE001
        return TestResult(
            name=test_name,
            passed=False,
            message=f"Unexpected error: {exc}",
        )


def run_traffic_manager_test(xodr_content: str, map_name: str) -> TestResult:
    """Test CARLA's Traffic Manager binary map cooking.

    Parses the .xodr with carla.Map() (offline) and then calls
    cook_in_memory_map() to generate the binary map used by the Traffic Manager.

    Args:
        xodr_content: Raw XML content of the .xodr file
        map_name: Name to assign to the map inside CARLA

    Returns:
        TestResult indicating pass/fail with the cooked binary map path/size
    """
    test_name = "Traffic Manager Test"
    try:
        import carla  # type: ignore[import]

        carla_map = carla.Map(map_name, xodr_content)
        with tempfile.TemporaryDirectory() as tmp_dir:
            bin_path = str(Path(tmp_dir) / f"{map_name}.bin")
            carla_map.cook_in_memory_map(bin_path)
            bin_file = Path(bin_path)
            if not bin_file.exists():
                return TestResult(
                    name=test_name,
                    passed=False,
                    message="cook_in_memory_map() returned but output file not found",
                    detail=f"Expected path: {bin_path}",
                )
            file_size = bin_file.stat().st_size
            detail = f"binary map path: {bin_path} ({file_size} bytes)"
            logger.debug("Traffic Manager test detail: %s", detail)
        return TestResult(
            name=test_name,
            passed=True,
            message="cook_in_memory_map() completed without errors",
            detail=detail,
        )
    except RuntimeError as exc:
        return TestResult(
            name=test_name,
            passed=False,
            message=f"CARLA RuntimeError: {exc}",
        )
    except ImportError as exc:
        return TestResult(
            name=test_name,
            passed=False,
            message=f"carla package not available: {exc}",
        )
    except Exception as exc:  # noqa: BLE001
        return TestResult(
            name=test_name,
            passed=False,
            message=f"Unexpected error: {exc}",
        )


def run_online_world_generation_test(
    xodr_content: str,
    host: str,
    port: int,
) -> TestResult:
    """Test CARLA world generation via a live server connection.

    Connects to a running CARLA server and calls generate_opendrive_world()
    to load the map. Requires a CARLA server to be running.

    Args:
        xodr_content: Raw XML content of the .xodr file
        host: Hostname or IP address of the CARLA server
        port: TCP port of the CARLA server

    Returns:
        TestResult indicating pass/fail with the generated world map name
    """
    test_name = "Online World Generation Test"
    try:
        import carla  # type: ignore[import]

        client = carla.Client(host, port)
        client.set_timeout(CONSTANTS.server_connection_timeout)
        params = carla.OpendriveGenerationParameters(
            vertex_distance=2.0,
            max_road_length=50.0,
            wall_height=0.0,
            additional_width=0.6,
            smooth_junctions=True,
            enable_mesh_visibility=True,
        )
        world = client.generate_opendrive_world(xodr_content, params)
        map_name = world.get_map().name
        detail = f"world map name: {map_name}"
        logger.debug("Online test detail: %s", detail)
        return TestResult(
            name=test_name,
            passed=True,
            message="generate_opendrive_world() succeeded",
            detail=detail,
        )
    except RuntimeError as exc:
        return TestResult(
            name=test_name,
            passed=False,
            message=f"CARLA RuntimeError: {exc}",
        )
    except ImportError as exc:
        return TestResult(
            name=test_name,
            passed=False,
            message=f"carla package not available: {exc}",
        )
    except Exception as exc:  # noqa: BLE001
        return TestResult(
            name=test_name,
            passed=False,
            message=f"Unexpected error: {exc}",
        )


# ---------------------------------------------------------------------------
# Report printing
# ---------------------------------------------------------------------------


def print_report(report: ImportTestReport) -> None:
    """Print a human-readable CARLA import test report to stdout.

    Args:
        report: The ImportTestReport to display
    """
    width = 72
    print("=" * width)
    print("  CARLA Import Test Report")
    print(f"  File   : {report.xodr_path}")
    print("=" * width)
    print()

    total = len(report.results)
    passed = report.passed_count
    failed = report.failed_count
    print(f"Tests : {total} total  |  {passed} passed  |  {failed} failed")

    for result in report.results:
        print()
        status = "PASS" if result.passed else "FAIL"
        print(f"[{status}] {result.name}")
        print(f"       {result.message}")
        if result.detail:
            print(f"       Detail: {result.detail}")

    print()
    print("-" * width)
    if report.all_passed:
        print(f"  RESULT: PASS  (all {total} tests passed)")
    else:
        print(f"  RESULT: FAIL  ({failed} of {total} tests failed)")
    print("=" * width)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    """Entry point for the carla-import-test command."""
    parser = argparse.ArgumentParser(
        prog="carla-import-test",
        description="Validate a .xodr file against CARLA's map loading pipeline.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "xodr_file",
        help="Path to the .xodr file to validate",
    )
    parser.add_argument(
        "--map-name",
        default=None,
        help="Name passed to carla.Map() (default: file stem)",
    )
    parser.add_argument(
        "--host",
        default=None,
        help="CARLA server host; enables the online world generation test when given",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=CONSTANTS.default_carla_port,
        help=f"CARLA server port (default: {CONSTANTS.default_carla_port})",
    )
    parser.add_argument(
        "--skip-tm",
        action="store_true",
        help="Skip the Traffic Manager cook test",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable DEBUG logging",
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    xodr_path = Path(args.xodr_file).resolve()
    if not xodr_path.exists():
        print(f"Error: file not found: {xodr_path}", file=sys.stderr)
        sys.exit(2)

    map_name: str = args.map_name if args.map_name else xodr_path.stem

    try:
        xodr_content = xodr_path.read_text(encoding="utf-8")
    except OSError as exc:
        print(f"Error: cannot read file: {exc}", file=sys.stderr)
        sys.exit(2)

    report = ImportTestReport(xodr_path=str(xodr_path))

    # Offline test 1: OpenDRIVE parser
    logger.debug("Running OpenDRIVE parser test for map '%s'", map_name)
    report.results.append(run_opendrive_parser_test(xodr_content, map_name))

    # Offline test 2: Traffic Manager (optional skip)
    if not args.skip_tm:
        logger.debug("Running Traffic Manager test for map '%s'", map_name)
        report.results.append(run_traffic_manager_test(xodr_content, map_name))

    # Online test 3: World generation (only when --host is provided)
    if args.host:
        logger.debug(
            "Running online world generation test (%s:%d)", args.host, args.port
        )
        report.results.append(
            run_online_world_generation_test(xodr_content, args.host, args.port)
        )

    print_report(report)

    sys.exit(0 if report.all_passed else 1)


if __name__ == "__main__":
    main()
