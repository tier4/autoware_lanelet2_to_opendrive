"""Singleton MapManager for holding map data used by coordinate transforms.

Only one LaneletMap and one RoadNetwork instance exist at any time.
Use MapManager.get_instance() to access the singleton.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, ClassVar, Optional

import lanelet2.io
import lanelet2.projection
from pyxodr.road_objects.network import RoadNetwork


class MapManager:
    """Singleton that holds one LaneletMap and one RoadNetwork instance.

    Usage::

        mm = MapManager.get_instance()
        mm.initialize(xodr_path=Path("map.xodr"), lanelet2_path=Path("map.osm"))

        # Access loaded maps
        lmap = mm.lanelet_map
        rnet = mm.road_network
    """

    _instance: ClassVar[Optional["MapManager"]] = None
    _lanelet_map: Optional[Any]
    _road_network: Optional[RoadNetwork]
    _geo_origin: Optional[tuple[float, float, float]]

    def __new__(cls) -> "MapManager":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._lanelet_map = None
            cls._instance._road_network = None
            cls._instance._geo_origin = None
        return cls._instance

    @classmethod
    def get_instance(cls) -> "MapManager":
        """Return the singleton MapManager instance."""
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    @classmethod
    def reset(cls) -> None:
        """Reset the singleton (for testing only)."""
        cls._instance = None

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def initialize(self, xodr_path: Path, lanelet2_path: Path) -> None:
        """Load both map files.

        Parameters
        ----------
        xodr_path:
            Path to the OpenDRIVE (.xodr) file.
        lanelet2_path:
            Path to the Lanelet2 map file (.osm or .xml).

        Raises
        ------
        RuntimeError
            If already initialized.
        FileNotFoundError
            If either map file does not exist.
        """
        if self._lanelet_map is not None or self._road_network is not None:
            raise RuntimeError(
                "MapManager is already initialized. "
                "Call MapManager.reset() before re-initializing (testing only)."
            )

        if not xodr_path.exists():
            raise FileNotFoundError(f"OpenDRIVE file not found: {xodr_path}")
        if not lanelet2_path.exists():
            raise FileNotFoundError(f"Lanelet2 file not found: {lanelet2_path}")

        # Parse geoReference from XODR to get the UTM origin
        xodr_content = xodr_path.read_text(encoding="utf-8")
        lat, lon, alt = _parse_geo_reference(xodr_content)
        self._geo_origin = (lat, lon, alt)

        # Load Lanelet2 map using the same origin as the XODR
        origin = lanelet2.io.Origin(lat, lon)
        projector = lanelet2.projection.UtmProjector(origin)
        self._lanelet_map = lanelet2.io.load(str(lanelet2_path), projector)

        # Load OpenDRIVE road network (pyxodr takes a file path, not content)
        self._road_network = RoadNetwork(str(xodr_path))

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def lanelet_map(self):  # type: ignore[return]
        """The loaded lanelet2.LaneletMap instance."""
        if self._lanelet_map is None:
            raise RuntimeError(
                "MapManager is not initialized. Call initialize() first."
            )
        return self._lanelet_map

    @property
    def road_network(self):  # type: ignore[return]
        """The loaded pyxodr.RoadNetwork instance."""
        if self._road_network is None:
            raise RuntimeError(
                "MapManager is not initialized. Call initialize() first."
            )
        return self._road_network

    @property
    def geo_origin(self) -> tuple[float, float, float]:
        """The (lat, lon, alt) origin parsed from the XODR geoReference."""
        if self._geo_origin is None:
            raise RuntimeError(
                "MapManager is not initialized. Call initialize() first."
            )
        return self._geo_origin


# ------------------------------------------------------------------
# Helper: parse geoReference PROJ string from XODR header
# ------------------------------------------------------------------


def _parse_geo_reference(xodr_content: str) -> tuple[float, float, float]:
    """Extract (lat, lon, alt) from the geoReference PROJ string in an XODR file.

    Parameters
    ----------
    xodr_content:
        Full text content of the .xodr file.

    Returns
    -------
    tuple[float, float, float]
        (latitude, longitude, altitude).  Altitude defaults to 0.0 if absent.

    Raises
    ------
    ValueError
        If lat_0 or lon_0 cannot be found.
    """
    # Extract the geoReference element content
    geo_ref_match = re.search(
        r"<geoReference>\s*<!\[CDATA\[(.*?)\]\]>\s*</geoReference>",
        xodr_content,
        re.DOTALL,
    )
    if geo_ref_match is None:
        # Fallback: try without CDATA wrapper
        geo_ref_match = re.search(
            r"<geoReference>(.*?)</geoReference>",
            xodr_content,
            re.DOTALL,
        )
    if geo_ref_match is None:
        raise ValueError("No <geoReference> element found in XODR file.")

    proj_string = geo_ref_match.group(1)

    lat_match = re.search(r"\+lat_0=([-\d.]+)", proj_string)
    lon_match = re.search(r"\+lon_0=([-\d.]+)", proj_string)

    if lat_match is None:
        raise ValueError(f"Could not find +lat_0 in geoReference: {proj_string!r}")
    if lon_match is None:
        raise ValueError(f"Could not find +lon_0 in geoReference: {proj_string!r}")

    lat = float(lat_match.group(1))
    lon = float(lon_match.group(1))

    alt_match = re.search(r"\+h_0=([-\d.]+)", proj_string)
    alt = float(alt_match.group(1)) if alt_match else 0.0

    return lat, lon, alt
