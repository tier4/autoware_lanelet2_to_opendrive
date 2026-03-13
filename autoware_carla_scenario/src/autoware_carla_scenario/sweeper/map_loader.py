"""Lightweight Lanelet2 map loader for pre-simulation use.

The sweeper runs *before* CARLA is started, so we cannot rely on
:class:`~autoware_carla_scenario.coordinate.map_manager.MapManager`
(which needs a CARLA world for z-offset computation).  This module
provides a minimal loader that only needs the Lanelet2 ``.osm`` and
OpenDRIVE ``.xodr`` file paths.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

# autoware_lanelet2_extension_python must be imported before lanelet2 to
# register Autoware-specific regulatory elements.
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2.core
import lanelet2.io

from ..coordinate.map_manager import _parse_geo_reference

logger = logging.getLogger(__name__)


def load_lanelet2_map(
    lanelet2_path: str | Path,
    xodr_path: str | Path,
) -> Any:
    """Load a Lanelet2 map using the geoReference from an XODR file.

    This is a lightweight alternative to :meth:`MapManager.initialize` that
    does *not* require a CARLA connection or RoadNetwork loading.

    Args:
        lanelet2_path: Path to the Lanelet2 ``.osm`` file.
        xodr_path: Path to the OpenDRIVE ``.xodr`` file (used only for the
            ``geoReference`` PROJ string).

    Returns:
        A ``lanelet2.core.LaneletMap`` instance.

    Raises:
        FileNotFoundError: If either file does not exist.
        ValueError: If the XODR file lacks a valid ``geoReference``.
    """
    lanelet2_path = Path(lanelet2_path)
    xodr_path = Path(xodr_path)

    if not lanelet2_path.exists():
        raise FileNotFoundError(f"Lanelet2 file not found: {lanelet2_path}")
    if not xodr_path.exists():
        raise FileNotFoundError(f"OpenDRIVE file not found: {xodr_path}")

    xodr_content = xodr_path.read_text(encoding="utf-8")
    lat, lon, _alt = _parse_geo_reference(xodr_content)

    origin = lanelet2.io.Origin(lat, lon)
    projector = MGRSProjector(origin)
    lanelet_map = lanelet2.io.load(str(lanelet2_path), projector)

    logger.info(
        "Loaded Lanelet2 map from %s (%d lanelets)",
        lanelet2_path,
        len(list(lanelet_map.laneletLayer)),
    )
    return lanelet_map
