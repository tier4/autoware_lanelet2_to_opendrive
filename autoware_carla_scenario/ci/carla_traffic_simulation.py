#!/usr/bin/env python3
"""CPU-only CARLA traffic-simulation smoke test for CI.

This script is a *self-contained* traffic-simulation harness intended to run
inside GitHub Actions on runners **without a GPU**.  It connects to a headless
CARLA 0.9.15 server (launched with ``-nullrhi`` so no graphics hardware is
required), loads an OpenDRIVE map produced by this repository's ``convert``
CLI via :meth:`carla.Client.generate_opendrive_world` (a procedural road mesh,
no pre-baked map assets needed), spawns a fleet of Traffic-Manager-controlled
vehicles, advances the world in synchronous mode, and asserts that the traffic
actually moved.

It deliberately does **not** import :mod:`autoware_carla_scenario`: that package
pins the CARLA 0.10.0 / 0.9.16 client and drives pre-baked town maps, neither of
which fits a CPU-only 0.9.15 server.  Keeping this harness dependency-free
(``carla`` + the standard library only) lets CI install just the matching
``carla==0.9.15`` client into a throwaway virtualenv.

Exit code is ``0`` when the traffic simulation passes and ``1`` otherwise, so it
can gate a CI job directly.

The world can come from a built-in CARLA town (``--map Town03``, which ships
with the image and needs no conversion) or from an OpenDRIVE file
(``--xodr map.xodr``).

Example::

    # Built-in town (default for CI)
    python carla_traffic_simulation.py \
        --map Town03 \
        --host localhost --port 2000 --tm-port 8000 \
        --num-vehicles 8 --ticks 120 \
        --output carla_traffic_sim_result.json

    # Or from an OpenDRIVE file
    python carla_traffic_simulation.py --xodr map.xodr
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List, Optional

import carla  # type: ignore[import]

logger = logging.getLogger("carla_traffic_simulation")


# ---------------------------------------------------------------------------
# Result reporting
# ---------------------------------------------------------------------------


@dataclass
class SimulationResult:
    """Machine-readable outcome of the traffic-simulation run."""

    passed: bool = False
    message: str = ""
    server_version: str = ""
    client_version: str = ""
    map_name: str = ""
    spawn_points: int = 0
    vehicles_spawned: int = 0
    vehicles_moved: int = 0
    total_distance_m: float = 0.0
    max_distance_m: float = 0.0
    ticks: int = 0
    elapsed_seconds: float = 0.0
    recorder_path: str = ""
    per_vehicle_distance_m: List[float] = field(default_factory=list)

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, ensure_ascii=False)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def wait_for_server(
    host: str, port: int, timeout_seconds: float, retry_interval: float
) -> carla.Client:
    """Poll the CARLA RPC port until the server answers a version handshake.

    A freshly started CARLA container opens its RPC port well before it can
    actually serve requests, so we retry the ``get_server_version`` handshake
    rather than a bare TCP connect.

    Raises:
        TimeoutError: If the server is not reachable within *timeout_seconds*.
    """
    deadline = time.monotonic() + timeout_seconds
    attempt = 0
    last_error: Optional[Exception] = None
    while time.monotonic() < deadline:
        attempt += 1
        try:
            client = carla.Client(host, port)
            client.set_timeout(10.0)
            server_version = client.get_server_version()
            logger.info(
                "Connected to CARLA server %s on %s:%d (attempt %d)",
                server_version,
                host,
                port,
                attempt,
            )
            return client
        except Exception as exc:  # noqa: BLE001 - broad on purpose while polling
            last_error = exc
            logger.info(
                "Waiting for CARLA server on %s:%d (attempt %d): %s",
                host,
                port,
                attempt,
                exc,
            )
            time.sleep(retry_interval)
    raise TimeoutError(
        f"CARLA server on {host}:{port} did not become ready within "
        f"{timeout_seconds:.0f}s (last error: {last_error})"
    )


def load_opendrive_world(client: carla.Client, xodr_path: Path) -> carla.World:
    """Generate a CARLA world from an OpenDRIVE file (no pre-baked assets).

    Uses :meth:`carla.Client.generate_opendrive_world`, which builds a
    procedural road mesh from the ``.xodr`` at runtime.  The *visual* mesh is
    disabled (``enable_mesh_visibility=False``) because rendering is off under
    ``-nullrhi``; the collision/physics mesh that vehicles drive on is always
    generated regardless.
    """
    xodr_content = xodr_path.read_text(encoding="utf-8")
    params = carla.OpendriveGenerationParameters(
        vertex_distance=2.0,
        max_road_length=500.0,
        wall_height=0.0,
        additional_width=0.6,
        smooth_junctions=True,
        enable_mesh_visibility=False,
        enable_pedestrian_navigation=False,
    )
    logger.info("Generating CARLA world from OpenDRIVE: %s", xodr_path)
    world = client.generate_opendrive_world(xodr_content, params)
    logger.info("OpenDRIVE world generated: %s", world.get_map().name)
    return world


def load_named_world(client: carla.Client, map_name: str) -> carla.World:
    """Load a built-in CARLA town map by name (e.g. ``Town03``).

    Built-in towns ship with the CARLA image and provide reliable spawn points
    and Traffic-Manager navigation, so no OpenDRIVE conversion is needed. Both
    short names (``Town03``) and full asset paths (``/Game/Carla/Maps/Town03``)
    are accepted. If the requested map is unavailable, the currently loaded
    world is used instead so CI degrades gracefully rather than erroring out.
    """
    available = client.get_available_maps()

    def _short(name: str) -> str:
        return name.split("/")[-1]

    target = next(
        (m for m in available if m == map_name or _short(m) == _short(map_name)),
        None,
    )
    if target is None:
        logger.warning(
            "Map %r is not available (have %s); using the current world",
            map_name,
            sorted(_short(m) for m in available),
        )
        return client.get_world()

    current = client.get_world().get_map().name
    if _short(current) == _short(target):
        logger.info("Requested map already loaded: %s", current)
        return client.get_world()

    logger.info("Loading built-in world: %s", target)
    return client.load_world(target)


def pick_spawn_transforms(world: carla.World, limit: int) -> List[carla.Transform]:
    """Return up to *limit* spawn transforms for the generated map.

    Prefers the map's recommended spawn points; falls back to raising sampled
    waypoints slightly above the road when none are available (which can happen
    for some converted OpenDRIVE files).
    """
    carla_map = world.get_map()
    spawn_points = list(carla_map.get_spawn_points())
    if spawn_points:
        return spawn_points[:limit]

    logger.warning("Map exposes no recommended spawn points; falling back to waypoints")
    transforms: List[carla.Transform] = []
    for waypoint in carla_map.generate_waypoints(5.0):
        transform = waypoint.transform
        transform.location.z += 0.5  # lift off the road to avoid collisions
        transforms.append(transform)
        if len(transforms) >= limit:
            break
    return transforms


def vehicle_blueprints(world: carla.World) -> List[carla.ActorBlueprint]:
    """Return four-wheeled vehicle blueprints suitable for autopilot traffic."""
    blueprint_library = world.get_blueprint_library()
    blueprints = [
        bp
        for bp in blueprint_library.filter("vehicle.*")
        if bp.has_attribute("number_of_wheels")
        and int(bp.get_attribute("number_of_wheels")) == 4
    ]
    # Fall back to every vehicle blueprint if the attribute filter is empty.
    return blueprints or list(blueprint_library.filter("vehicle.*"))


def spawn_traffic(
    client: carla.Client,
    world: carla.World,
    tm_port: int,
    num_vehicles: int,
    seed: int,
) -> List[int]:
    """Batch-spawn autopilot vehicles and return the spawned actor IDs."""
    spawn_transforms = pick_spawn_transforms(world, num_vehicles)
    blueprints = vehicle_blueprints(world)
    if not spawn_transforms:
        raise RuntimeError("No spawn transforms available on the generated map")
    if not blueprints:
        raise RuntimeError("No vehicle blueprints available on the server")

    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    batch = []
    for index, transform in enumerate(spawn_transforms):
        blueprint = blueprints[index % len(blueprints)]
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "autopilot")
        # Deterministic colour selection keeps runs reproducible.
        if blueprint.has_attribute("color"):
            colors = blueprint.get_attribute("color").recommended_values
            if colors:
                blueprint.set_attribute("color", colors[(index + seed) % len(colors)])
        batch.append(
            SpawnActor(blueprint, transform).then(
                SetAutopilot(FutureActor, True, tm_port)
            )
        )

    actor_ids: List[int] = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logger.debug("Spawn skipped: %s", response.error)
        else:
            actor_ids.append(response.actor_id)

    logger.info(
        "Spawned %d/%d vehicles with autopilot enabled",
        len(actor_ids),
        len(spawn_transforms),
    )
    return actor_ids


def _distance(a: carla.Location, b: carla.Location) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


# ---------------------------------------------------------------------------
# Main simulation routine
# ---------------------------------------------------------------------------


def run_simulation(args: argparse.Namespace) -> SimulationResult:
    """Run the full CPU-only traffic simulation and return its result."""
    result = SimulationResult()
    start = time.monotonic()

    if not args.map and not args.xodr:
        result.message = "Either --map or --xodr must be provided"
        return result

    xodr_path: Optional[Path] = None
    if args.xodr:
        xodr_path = Path(args.xodr)
        if not xodr_path.is_file():
            result.message = f"OpenDRIVE file not found: {xodr_path}"
            return result

    client = wait_for_server(
        args.host, args.port, args.connect_timeout, args.retry_interval
    )
    client.set_timeout(args.rpc_timeout)
    result.client_version = client.get_client_version()
    result.server_version = client.get_server_version()

    # Prefer a built-in town map when requested; otherwise build one from the
    # OpenDRIVE file.
    if args.map:
        world = load_named_world(client, args.map)
    else:
        assert xodr_path is not None  # noqa: S101 - guarded above
        world = load_opendrive_world(client, xodr_path)
    result.map_name = world.get_map().name
    result.spawn_points = len(world.get_map().get_spawn_points())

    original_settings = world.get_settings()
    traffic_manager = client.get_trafficmanager(args.tm_port)

    actor_ids: List[int] = []
    recorder_started = False
    try:
        # Deterministic synchronous simulation.
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = args.fixed_delta
        world.apply_settings(settings)

        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(args.seed)

        # Start the native CARLA recorder before spawning so the whole episode
        # (spawns included) is captured. The path is resolved on the *server*
        # side, so it must be a path inside the CARLA container. The resulting
        # .log is replayable via ``client.replay_file(...)``.
        if args.recorder_path:
            try:
                info = client.start_recorder(args.recorder_path, True)
                recorder_started = True
                result.recorder_path = args.recorder_path
                logger.info(
                    "Started CARLA recorder at %s: %s", args.recorder_path, info
                )
            except Exception:  # noqa: BLE001
                logger.warning(
                    "Failed to start CARLA recorder at %s",
                    args.recorder_path,
                    exc_info=True,
                )

        actor_ids = spawn_traffic(
            client, world, args.tm_port, args.num_vehicles, args.seed
        )
        result.vehicles_spawned = len(actor_ids)
        if not actor_ids:
            result.message = "No vehicles could be spawned on the map"
            return result

        vehicles = [world.get_actor(actor_id) for actor_id in actor_ids]

        # Warm-up ticks let physics settle before we record start positions.
        for _ in range(args.warmup_ticks):
            world.tick()
        start_locations = [vehicle.get_location() for vehicle in vehicles]

        for _ in range(args.ticks):
            world.tick()
        end_locations = [vehicle.get_location() for vehicle in vehicles]

        distances = [
            _distance(start, end) for start, end in zip(start_locations, end_locations)
        ]
        result.ticks = args.ticks
        result.per_vehicle_distance_m = [round(d, 3) for d in distances]
        result.total_distance_m = round(sum(distances), 3)
        result.max_distance_m = round(max(distances), 3) if distances else 0.0
        result.vehicles_moved = sum(
            1 for d in distances if d >= args.per_vehicle_min_distance
        )

        # Pass criteria: at least one vehicle drove a meaningful distance and
        # the fleet as a whole covered the configured minimum ground.
        passed = (
            result.vehicles_moved >= args.min_moved_vehicles
            and result.total_distance_m >= args.min_total_distance
        )
        result.passed = passed
        result.message = (
            f"{result.vehicles_moved}/{result.vehicles_spawned} vehicles moved, "
            f"total {result.total_distance_m:.1f} m over {result.ticks} ticks "
            f"({result.ticks * args.fixed_delta:.1f} s simulated)"
        )
    finally:
        # Stop the recorder first so the .log is flushed before we tear the
        # scene down (best-effort; failures here must not mask results).
        if recorder_started:
            try:
                client.stop_recorder()
                logger.info("Stopped CARLA recorder: %s", result.recorder_path)
            except Exception:  # noqa: BLE001
                logger.debug("Failed to stop CARLA recorder", exc_info=True)
        # Restore async mode and destroy spawned actors so the server is left
        # in a clean state (best-effort; failures here must not mask results).
        try:
            traffic_manager.set_synchronous_mode(False)
        except Exception:  # noqa: BLE001
            logger.debug("Failed to reset traffic manager", exc_info=True)
        try:
            world.apply_settings(original_settings)
        except Exception:  # noqa: BLE001
            logger.debug("Failed to restore world settings", exc_info=True)
        if actor_ids:
            client.apply_batch(
                [carla.command.DestroyActor(actor_id) for actor_id in actor_ids]
            )

    result.elapsed_seconds = round(time.monotonic() - start, 2)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CPU-only CARLA traffic-simulation smoke test for CI.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--map",
        default="",
        help=(
            "Built-in CARLA town map to load (e.g. Town03). Ships with the "
            "CARLA image, so no OpenDRIVE conversion is needed. Mutually "
            "exclusive with --xodr; one of the two is required."
        ),
    )
    parser.add_argument(
        "--xodr",
        default="",
        help=(
            "Path to an OpenDRIVE (.xodr) map file to generate a world from. "
            "Used when --map is not given."
        ),
    )
    parser.add_argument("--host", default="localhost", help="CARLA RPC host.")
    parser.add_argument("--port", type=int, default=2000, help="CARLA RPC port.")
    parser.add_argument(
        "--tm-port", type=int, default=8000, help="CARLA Traffic Manager port."
    )
    parser.add_argument(
        "--num-vehicles", type=int, default=20, help="Number of vehicles to spawn."
    )
    parser.add_argument(
        "--ticks",
        type=int,
        default=200,
        help="Number of synchronous ticks to simulate after warm-up.",
    )
    parser.add_argument(
        "--warmup-ticks",
        type=int,
        default=20,
        help="Ticks to run before recording start positions.",
    )
    parser.add_argument(
        "--fixed-delta",
        type=float,
        default=0.05,
        help="Fixed simulation step in seconds (0.05 = 20 Hz).",
    )
    parser.add_argument(
        "--min-moved-vehicles",
        type=int,
        default=1,
        help="Minimum vehicles that must move for the run to pass.",
    )
    parser.add_argument(
        "--min-total-distance",
        type=float,
        default=5.0,
        help="Minimum total distance (m) the fleet must cover to pass.",
    )
    parser.add_argument(
        "--per-vehicle-min-distance",
        type=float,
        default=0.5,
        help="Distance (m) above which a single vehicle counts as 'moved'.",
    )
    parser.add_argument("--seed", type=int, default=0, help="Traffic Manager seed.")
    parser.add_argument(
        "--recorder-path",
        default="",
        help=(
            "Server-side path for the native CARLA replay log (.log). Resolved "
            "inside the CARLA server container. Empty disables recording."
        ),
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=300.0,
        help="Seconds to wait for the CARLA server to become ready.",
    )
    parser.add_argument(
        "--retry-interval",
        type=float,
        default=5.0,
        help="Seconds between server readiness retries.",
    )
    parser.add_argument(
        "--rpc-timeout",
        type=float,
        default=120.0,
        help=(
            "Per-request RPC timeout in seconds. Generous so loading a built-in "
            "town map on a CPU-only server does not time out."
        ),
    )
    parser.add_argument(
        "--output",
        default="carla_traffic_sim_result.json",
        help="Path to write the JSON result report.",
    )
    return parser


def main() -> None:
    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s %(name)s: %(message)s"
    )
    args = build_parser().parse_args()

    try:
        result = run_simulation(args)
    except Exception as exc:  # noqa: BLE001 - report any failure as a FAIL result
        logger.exception("Traffic simulation raised an unexpected error")
        result = SimulationResult(passed=False, message=f"Exception: {exc}")

    output_path = Path(args.output)
    output_path.write_text(result.to_json(), encoding="utf-8")

    status = "PASS" if result.passed else "FAIL"
    print("=" * 60)  # noqa: T201
    print(f"[{status}] CARLA traffic simulation")  # noqa: T201
    print(f"  server        : {result.server_version}")  # noqa: T201
    print(f"  client        : {result.client_version}")  # noqa: T201
    print(f"  map           : {result.map_name}")  # noqa: T201
    print(f"  spawn points  : {result.spawn_points}")  # noqa: T201
    print(f"  vehicles      : {result.vehicles_spawned}")  # noqa: T201
    print(f"  vehicles moved: {result.vehicles_moved}")  # noqa: T201
    print(f"  total distance: {result.total_distance_m:.1f} m")  # noqa: T201
    print(f"  max distance  : {result.max_distance_m:.1f} m")  # noqa: T201
    print(f"  message       : {result.message}")  # noqa: T201
    if result.recorder_path:
        print(f"  replay log    : {result.recorder_path} (server-side)")  # noqa: T201
    print(f"  result JSON   : {output_path.resolve()}")  # noqa: T201
    print("=" * 60)  # noqa: T201

    sys.exit(0 if result.passed else 1)


if __name__ == "__main__":
    main()
