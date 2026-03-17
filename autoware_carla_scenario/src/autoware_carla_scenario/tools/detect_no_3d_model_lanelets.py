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

import carla
import lanelet2.core
import lanelet2.geometry
import lanelet2.io
import numpy as np
from autoware_lanelet2_extension_python.projection import MGRSProjector
from omegaconf import DictConfig, OmegaConf
from tqdm import tqdm

from autoware_carla_scenario.coordinate.map_manager import (
    _parse_geo_reference,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _interpolate_at_s(
    points: list[tuple[float, float, float]], s: float
) -> tuple[float, float, float]:
    """Interpolate (x, y, z) at arc length *s* along a 3D polyline."""
    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])
    zs = np.array([p[2] for p in points])

    pts_2d = np.column_stack([xs, ys])
    diffs = np.diff(pts_2d, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    arc = np.zeros(len(points))
    arc[1:] = np.cumsum(seg_lengths)

    s_clamped = float(np.clip(s, arc[0], arc[-1]))
    x = float(np.interp(s_clamped, arc, xs))
    y = float(np.interp(s_clamped, arc, ys))
    z = float(np.interp(s_clamped, arc, zs))

    return x, y, z


# ---------------------------------------------------------------------------
# Z-offset computation
# ---------------------------------------------------------------------------


def _compute_z_offset(
    lanelet_map: lanelet2.core.LaneletMap,
    mgrs_offset: tuple[float, float],
    world: carla.World,
) -> float:
    """Compute ``lanelet2_z - carla_z`` using CARLA spawn points.

    Averages the per-point offset ``(lanelet2_z - carla_spawn_z)`` across
    all spawn points that are within 10 m of a lanelet centerline.
    """
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        logger.warning("No spawn points found; using z_offset=0.0")
        return 0.0

    offset_x, offset_y = mgrs_offset
    offsets: list[float] = []

    for sp in spawn_points:
        mgrs_x = sp.location.x + offset_x
        mgrs_y = -sp.location.y + offset_y

        query = lanelet2.core.BasicPoint2d(mgrs_x, mgrs_y)
        results = lanelet2.geometry.findNearest(lanelet_map.laneletLayer, query, 1)
        if not results:
            continue

        if results[0][0] > 10.0:
            continue

        lanelet = results[0][1]
        best_d2 = float("inf")
        ll2_z = 0.0
        for pt in lanelet.centerline:
            d2 = (pt.x - mgrs_x) ** 2 + (pt.y - mgrs_y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                ll2_z = pt.z

        offsets.append(ll2_z - sp.location.z)

    if not offsets:
        logger.warning("No usable spawn points for z_offset; using 0.0")
        return 0.0

    z_offset = float(np.mean(offsets))
    logger.info("Computed z_offset=%.4f from %d spawn points", z_offset, len(offsets))
    return z_offset


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
        mgrs_x, mgrs_y, mgrs_z = _interpolate_at_s(points, s_mid)

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


def _resolve_map_paths(yaml_path: Path) -> tuple[Path, Path]:
    """Read ``xodr_path`` and ``lanelet2_path`` from a map configuration YAML.

    OmegaConf resolves ``${oc.env:VAR,default}`` interpolations automatically.

    Returns
    -------
    tuple[Path, Path]
        ``(xodr_path, lanelet2_path)`` as ``Path`` objects.

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

    return xodr_path, ll2_path


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
        xodr_path, lanelet2_path = _resolve_map_paths(args.yaml_path)
    except (ValueError, FileNotFoundError) as exc:
        logger.error("%s", exc)
        sys.exit(1)
    logger.info("XODR:     %s", xodr_path)
    logger.info("Lanelet2: %s", lanelet2_path)

    # ------------------------------------------------------------------
    # Connect to CARLA
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

    world = client.get_world()

    # ------------------------------------------------------------------
    # Load maps and compute offsets
    # ------------------------------------------------------------------
    xodr_content = xodr_path.read_text(encoding="utf-8")
    lat, lon, alt = _parse_geo_reference(xodr_content)
    logger.info("geoReference: lat=%.6f, lon=%.6f, alt=%.2f", lat, lon, alt)

    origin = lanelet2.io.Origin(lat, lon)
    projector = MGRSProjector(origin)
    lanelet_map = lanelet2.io.load(str(lanelet2_path), projector)
    logger.info(
        "Loaded Lanelet2 map with %d lanelets",
        len(lanelet_map.laneletLayer),
    )

    fwd = projector.forward(lanelet2.core.GPSPoint(lat, lon, alt))
    mgrs_offset = (fwd.x, fwd.y)
    logger.info("MGRS offset: (%.4f, %.4f)", mgrs_offset[0], mgrs_offset[1])

    z_offset = _compute_z_offset(lanelet_map, mgrs_offset, world)

    # ------------------------------------------------------------------
    # Detect lanelets without 3D model
    # ------------------------------------------------------------------
    no_model_ids = detect_no_3d_model_lanelets(
        world=world,
        lanelet_map=lanelet_map,
        mgrs_offset=mgrs_offset,
        z_offset=z_offset,
        ray_distance_upper=args.ray_upper,
        ray_distance_lower=args.ray_lower,
    )

    total = len(lanelet_map.laneletLayer)
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
