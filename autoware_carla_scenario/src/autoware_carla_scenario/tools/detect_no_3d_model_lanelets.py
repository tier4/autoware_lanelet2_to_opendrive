"""Detect lanelets without a matching 3D model in CARLA.

For each lanelet in the Lanelet2 map, this script:

1. Computes the midpoint position (s = length / 2, t = 0) on the centerline.
2. Converts the position to CARLA world coordinates.
3. Performs a downward ground projection (ray cast) at that position.
4. Records the lanelet ID if no ground hit is detected.

The detected lanelet IDs are written to the map configuration YAML file
under ``no_3d_model_lanelet_ids``.

The map configuration YAML (e.g. ``nishishinjuku.yaml``) already contains
``xodr_path`` and ``lanelet2_path``; they are resolved via OmegaConf so
that ``${oc.env:...}`` interpolations work transparently.

Usage::

    uv run detect-no-3d-model \\
        autoware_carla_scenario/src/autoware_carla_scenario/examples/conf/map/nishishinjuku.yaml

    # With verbose logging
    uv run detect-no-3d-model -v \\
        autoware_carla_scenario/src/autoware_carla_scenario/examples/conf/map/nishishinjuku.yaml

Coordinate systems
------------------
Lanelet2 (MGRS absolute):  x = East, y = North, z = Up
CARLA world (left-hand):   x = East, y = South, z = Up

    carla_x =   (mgrs_x - mgrs_offset_x)
    carla_y = -(mgrs_y - mgrs_offset_y)
    carla_z =   mgrs_z - z_offset
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import os
import shutil

import carla
import lanelet2.geometry
from omegaconf import DictConfig, OmegaConf
from tqdm import tqdm

from autoware_carla_scenario.coordinate.map_manager import MapManager
from autoware_carla_scenario.coordinate.transform import (
    _interpolate_at_s as _interpolate_at_s_4,
)
from autoware_carla_scenario.scenario_runner import _map_name_to_env_var

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Main detection logic
# ---------------------------------------------------------------------------


def detect_no_3d_model_lanelets(
    world: carla.World,
    lanelet_map: lanelet2.core.LaneletMap,
    mgrs_offset: tuple[float, float],
    z_offset: float,
    *,
    ray_distance_upper: float = 5.0,
    ray_distance_lower: float = 5.0,
) -> list[int]:
    """Return sorted list of lanelet IDs that have no ground geometry in CARLA.

    For each lanelet, the midpoint (s = length / 2, t = 0) is computed and
    converted to CARLA world coordinates.  A downward ray is cast from
    ``z_estimate + ray_distance_upper`` with a total search distance of
    ``ray_distance_upper + ray_distance_lower``.

    - If the ray hits nothing (``result is None``), the lanelet is recorded.
    - If the ray hits geometry with ``CityObjectLabel.NONE``, a warning is
      logged but the lanelet is **not** recorded (geometry exists, just no
      semantic tag).
    - If ``ground_projection`` raises ``RuntimeError`` (mesh not loaded),
      the lanelet is recorded.

    Parameters
    ----------
    world:
        An active ``carla.World`` instance.
    lanelet_map:
        A loaded ``lanelet2.LaneletMap`` instance.
    mgrs_offset:
        ``(offset_x, offset_y)`` for converting MGRS to XODR coordinates.
    z_offset:
        ``lanelet2_z - carla_z`` vertical offset.
    ray_distance_upper:
        Search range (m) above the estimated z.
    ray_distance_lower:
        Search range (m) below the estimated z.

    Returns
    -------
    list[int]
        Sorted lanelet IDs where ground projection found no hit.
    """
    no_model_ids: list[int] = []
    total = len(lanelet_map.laneletLayer)

    for lanelet in tqdm(
        lanelet_map.laneletLayer, total=total, desc="Checking lanelets"
    ):
        length = lanelet2.geometry.length2d(lanelet)
        if length < 1e-6:
            logger.warning("Lanelet %d has near-zero length, skipping", lanelet.id)
            continue

        points = [(p.x, p.y, p.z) for p in lanelet.centerline]
        s_mid = length / 2.0
        mgrs_x, mgrs_y, mgrs_z, _ = _interpolate_at_s_4(points, s_mid)

        # MGRS -> CARLA world coordinates
        carla_x = mgrs_x - mgrs_offset[0]
        carla_y = -(mgrs_y - mgrs_offset[1])
        carla_z = mgrs_z - z_offset

        origin = carla.Location(
            x=carla_x,
            y=carla_y,
            z=carla_z + ray_distance_upper,
        )
        search_distance = ray_distance_upper + ray_distance_lower

        try:
            result = world.ground_projection(origin, search_distance)
        except AttributeError:
            raise RuntimeError(
                "world.ground_projection() is not available in this CARLA version. "
                "A CARLA build that supports ground_projection is required."
            ) from None
        except RuntimeError as exc:
            logger.warning(
                "ground_projection raised RuntimeError for lanelet %d "
                "at (%.2f, %.2f, %.2f): %s",
                lanelet.id,
                carla_x,
                carla_y,
                carla_z,
                exc,
            )
            no_model_ids.append(lanelet.id)
            continue

        if result is None:
            logger.info(
                "No ground hit for lanelet %d at (%.2f, %.2f, %.2f)",
                lanelet.id,
                carla_x,
                carla_y,
                carla_z,
            )
            no_model_ids.append(lanelet.id)
            continue

        if result.label == carla.CityObjectLabel.NONE:
            logger.warning(
                "Ground hit at lanelet %d has CityObjectLabel.NONE "
                "at (%.2f, %.2f); geometry exists but has no semantic tag",
                lanelet.id,
                carla_x,
                carla_y,
            )

        logger.debug(
            "Lanelet %d: hit z=%.3f (estimate=%.3f, delta=%.3f)",
            lanelet.id,
            result.location.z,
            carla_z,
            result.location.z - carla_z,
        )

    return sorted(no_model_ids)


# ---------------------------------------------------------------------------
# YAML update
# ---------------------------------------------------------------------------


def _update_yaml(yaml_path: Path, lanelet_ids: list[int]) -> None:
    """Update ``no_3d_model_lanelet_ids`` in the map configuration YAML.

    Uses line-level manipulation to preserve comments, Hydra interpolation
    strings (``${...}``), and other formatting.
    """
    content = yaml_path.read_text(encoding="utf-8")
    lines = content.splitlines(keepends=True)

    output: list[str] = []
    i = 0
    found = False

    while i < len(lines):
        line = lines[i]
        stripped = line.lstrip()

        if stripped.startswith("no_3d_model_lanelet_ids:"):
            found = True
            indent = line[: len(line) - len(stripped)]

            if lanelet_ids:
                output.append(f"{indent}no_3d_model_lanelet_ids:\n")
                for lid in lanelet_ids:
                    output.append(f"{indent}  - {lid}\n")
            else:
                output.append(f"{indent}no_3d_model_lanelet_ids: []\n")

            # Skip any existing block-list items on subsequent lines
            i += 1
            child_indent = indent + "  "
            while i < len(lines):
                next_stripped = lines[i].lstrip()
                if next_stripped.startswith("- ") and lines[i].startswith(child_indent):
                    i += 1
                else:
                    break
            continue

        output.append(line)
        i += 1

    if not found:
        logger.warning(
            "no_3d_model_lanelet_ids key not found in %s; appending under map:",
            yaml_path,
        )
        if lanelet_ids:
            output.append("  no_3d_model_lanelet_ids:\n")
            for lid in lanelet_ids:
                output.append(f"    - {lid}\n")
        else:
            output.append("  no_3d_model_lanelet_ids: []\n")

    yaml_path.write_text("".join(output), encoding="utf-8")


# ---------------------------------------------------------------------------
# YAML path resolution
# ---------------------------------------------------------------------------


def _resolve_map_config(yaml_path: Path) -> tuple[str, Path, Path]:
    """Read ``name``, ``xodr_path``, and ``lanelet2_path`` from a map config YAML.

    OmegaConf resolves ``${oc.env:VAR,default}`` interpolations automatically.

    Returns
    -------
    tuple[str, Path, Path]
        ``(map_name, xodr_path, lanelet2_path)``.

    Raises
    ------
    ValueError
        If the YAML structure is unexpected or required keys are missing.
    FileNotFoundError
        If either resolved file does not exist.
    """
    raw = OmegaConf.load(yaml_path)
    if not isinstance(raw, DictConfig):
        raise ValueError(f"Expected a YAML mapping in {yaml_path}, got a sequence")

    # The YAML may have the paths nested under a ``map:`` key (Hydra
    # package style) or at the top level.
    map_cfg = raw.get("map", raw)
    if not isinstance(map_cfg, DictConfig):
        raise ValueError(
            f"Expected 'map' to be a mapping in {yaml_path}, "
            f"got {type(map_cfg).__name__}"
        )

    map_name = map_cfg.get("name")
    if map_name is None:
        raise ValueError(f"map.name not found in {yaml_path}")

    xodr_raw = map_cfg.get("xodr_path")
    ll2_raw = map_cfg.get("lanelet2_path")

    if xodr_raw is None:
        raise ValueError(f"xodr_path not found in {yaml_path}")
    if ll2_raw is None:
        raise ValueError(f"lanelet2_path not found in {yaml_path}")

    xodr_path = Path(str(xodr_raw))
    ll2_path = Path(str(ll2_raw))

    if not xodr_path.exists():
        raise FileNotFoundError(
            f"XODR file not found: {xodr_path} (resolved from {yaml_path})"
        )
    if not ll2_path.exists():
        raise FileNotFoundError(
            f"Lanelet2 file not found: {ll2_path} (resolved from {yaml_path})"
        )

    return str(map_name), xodr_path, ll2_path


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main() -> None:
    """CLI entry point for detecting lanelets without a CARLA 3D model."""
    parser = argparse.ArgumentParser(
        description=(
            "Detect lanelets without a matching 3D model in CARLA and "
            "write the IDs to a map configuration YAML file.  "
            "The xodr_path and lanelet2_path are read from the YAML "
            "config automatically."
        ),
    )
    parser.add_argument(
        "yaml_path",
        type=Path,
        help="Path to the map configuration YAML (e.g. conf/map/nishishinjuku.yaml)",
    )
    parser.add_argument(
        "--host",
        default="localhost",
        help="CARLA server hostname (default: localhost)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=2000,
        help="CARLA server port (default: 2000)",
    )
    parser.add_argument(
        "--ray-upper",
        type=float,
        default=5.0,
        help="Ray search range above z estimate in metres (default: 5.0)",
    )
    parser.add_argument(
        "--ray-lower",
        type=float,
        default=5.0,
        help="Ray search range below z estimate in metres (default: 5.0)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="CARLA client timeout in seconds (default: 10.0)",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose (DEBUG) logging",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # ------------------------------------------------------------------
    # Resolve map file paths from YAML
    # ------------------------------------------------------------------
    if not args.yaml_path.exists():
        logger.error("YAML config not found: %s", args.yaml_path)
        sys.exit(1)

    try:
        map_name, xodr_path, lanelet2_path = _resolve_map_config(args.yaml_path)
    except (ValueError, FileNotFoundError) as exc:
        logger.error("%s", exc)
        sys.exit(1)
    logger.info("Map name: %s", map_name)
    logger.info("XODR:     %s", xodr_path)
    logger.info("Lanelet2: %s", lanelet2_path)

    # ------------------------------------------------------------------
    # Connect to CARLA and load the map
    # ------------------------------------------------------------------
    logger.info("Connecting to CARLA at %s:%d ...", args.host, args.port)
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    try:
        version = client.get_server_version()
        logger.info("Connected to CARLA server version %s", version)
    except RuntimeError as exc:
        logger.error("Cannot connect to CARLA server: %s", exc)
        sys.exit(1)

    # Overwrite the CARLA-internal XODR and load the map (same as
    # ScenarioRunner.load_map_by_overwriting_xodr).
    env_var = _map_name_to_env_var(map_name)
    dest_str = os.environ.get(env_var)
    if dest_str:
        dest = Path(dest_str)
        logger.info("Overwriting CARLA XODR: %s -> %s", xodr_path, dest)
        shutil.copy2(xodr_path, dest)
    else:
        logger.info(
            "Environment variable %s not set; loading map without XODR overwrite",
            env_var,
        )

    logger.info("Loading CARLA map %s ...", map_name)
    world = client.load_world(map_name)

    # ------------------------------------------------------------------
    # Initialize MapManager (loads lanelet2, computes mgrs_offset & z_offset)
    # ------------------------------------------------------------------
    MapManager.reset()
    mm = MapManager.get_instance()
    mm.initialize(
        xodr_path=xodr_path,
        lanelet2_path=lanelet2_path,
        carla_world=world,
    )
    logger.info(
        "Loaded Lanelet2 map with %d lanelets",
        len(mm.lanelet_map.laneletLayer),
    )
    logger.info("MGRS offset: (%.4f, %.4f)", mm.mgrs_offset[0], mm.mgrs_offset[1])
    logger.info("z_offset: %.4f", mm.z_offset)

    # ------------------------------------------------------------------
    # Detect lanelets without 3D model
    # ------------------------------------------------------------------
    no_model_ids = detect_no_3d_model_lanelets(
        world=world,
        lanelet_map=mm.lanelet_map,
        mgrs_offset=mm.mgrs_offset,
        z_offset=mm.z_offset,
        ray_distance_upper=args.ray_upper,
        ray_distance_lower=args.ray_lower,
    )

    total = len(mm.lanelet_map.laneletLayer)
    logger.info(
        "Detection complete: %d / %d lanelets have no 3D model",
        len(no_model_ids),
        total,
    )
    if no_model_ids:
        logger.info("Lanelet IDs without 3D model: %s", no_model_ids)

    # ------------------------------------------------------------------
    # Update YAML config
    # ------------------------------------------------------------------
    _update_yaml(args.yaml_path, no_model_ids)
    logger.info("Updated %s with %d lanelet IDs", args.yaml_path, len(no_model_ids))


if __name__ == "__main__":
    main()
