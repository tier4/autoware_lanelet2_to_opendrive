"""Singleton MapManager for holding map data used by coordinate transforms.

Only one LaneletMap and one RoadNetwork instance exist at any time.
Use MapManager.get_instance() to access the singleton.

Coordinate note
---------------
MGRSProjector returns *absolute* MGRS coordinates, while the OpenDRIVE reference
line stores coordinates *relative* to the geoReference origin (lat_0, lon_0).
The MGRS offset corrects for this:

    xodr_xy = mgrs_xy - mgrs_offset
    mgrs_xy = xodr_xy + mgrs_offset

where ``mgrs_offset = MGRSProjector.forward(lat_0, lon_0)``.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, ClassVar, Optional

# autoware_lanelet2_extension_python must be imported before lanelet2 to register
# Autoware-specific regulatory elements (road_marking, detection_area, etc.)
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2.core
import lanelet2.io
from pyxodr.road_objects.network import RoadNetwork


class MapManager:
    """Singleton that holds one LaneletMap and one RoadNetwork instance.

    Usage::

        mm = MapManager.get_instance()
        mm.initialize(xodr_path=Path("map.xodr"), lanelet2_path=Path("map.osm"))

        # Access loaded maps
        lmap = mm.lanelet_map
        rnet = mm.road_network

        # MGRS ↔ XODR coordinate offset
        ox, oy = mm.mgrs_offset
    """

    _instance: ClassVar[Optional["MapManager"]] = None
    _lanelet_map: Optional[Any]
    _road_network: Optional[RoadNetwork]
    _geo_origin: Optional[tuple[float, float, float]]
    _mgrs_offset: Optional[tuple[float, float]]
    _z_offset: Optional[float]

    def __new__(cls) -> "MapManager":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._lanelet_map = None
            cls._instance._road_network = None
            cls._instance._geo_origin = None
            cls._instance._mgrs_offset = None
            cls._instance._z_offset = None
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
        projector = MGRSProjector(origin)
        self._lanelet_map = lanelet2.io.load(str(lanelet2_path), projector)

        # Compute MGRS offset: forward-project the geoReference origin.
        # MGRSProjector returns absolute MGRS coords, while XODR stores coords
        # relative to the geoReference origin, so we need this correction.
        fwd = projector.forward(lanelet2.core.GPSPoint(lat, lon, alt))
        self._mgrs_offset = (fwd.x, fwd.y)

        # Load OpenDRIVE road network (pyxodr takes a file path, not content)
        self._road_network = RoadNetwork(str(xodr_path))

        # Compute vertical offset: Lanelet2 z (MGRS absolute elevation) minus
        # XODR z (elevation relative to geoReference origin).  This is constant
        # across the map and lets us convert z between the two systems.
        self._z_offset = self._compute_z_offset()

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
    def road_network(self) -> RoadNetwork:
        """The loaded pyxodr.RoadNetwork instance (roads are pre-loaded)."""
        if self._road_network is None:
            raise RuntimeError(
                "MapManager is not initialized. Call initialize() first."
            )
        if not self._road_network.road_ids_to_object:
            self._road_network.get_roads()
        return self._road_network

    @property
    def geo_origin(self) -> tuple[float, float, float]:
        """The (lat, lon, alt) origin parsed from the XODR geoReference."""
        if self._geo_origin is None:
            raise RuntimeError(
                "MapManager is not initialized. Call initialize() first."
            )
        return self._geo_origin

    @property
    def mgrs_offset(self) -> tuple[float, float]:
        """(offset_x, offset_y) to convert between MGRS and XODR coordinates.

        xodr_xy = mgrs_xy - mgrs_offset
        mgrs_xy = xodr_xy + mgrs_offset
        """
        if self._mgrs_offset is None:
            raise RuntimeError(
                "MapManager is not initialized. Call initialize() first."
            )
        return self._mgrs_offset

    @property
    def z_offset(self) -> float:
        """Vertical offset: ``lanelet2_z - xodr_z``.

        Use this to convert between Lanelet2 absolute elevation and XODR/CARLA
        relative elevation::

            carla_z = lanelet2_z - z_offset
            lanelet2_z = carla_z + z_offset
        """
        if self._z_offset is None:
            raise RuntimeError(
                "MapManager is not initialized. Call initialize() first."
            )
        return self._z_offset

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _compute_z_offset(self) -> float:
        """Compute ``lanelet2_z − xodr_z`` by sampling one reference point.

        Takes the first lanelet centerline point, finds the nearest XODR road
        reference-line point at the same (x, y), and returns the difference
        in their z coordinates.  The offset is assumed constant across the map.
        """
        import numpy as np

        # Sample the first available lanelet centerline point
        assert self._lanelet_map is not None
        for lanelet in self._lanelet_map.laneletLayer:
            ref_pt = lanelet.centerline[0]
            break
        else:
            return 0.0

        ll2_z = ref_pt.z
        # Convert Lanelet2 MGRS (x, y) → XODR-relative (x, y)
        assert self._mgrs_offset is not None
        xodr_x = ref_pt.x - self._mgrs_offset[0]
        xodr_y = ref_pt.y - self._mgrs_offset[1]

        # Ensure roads are loaded
        assert self._road_network is not None
        if not self._road_network.road_ids_to_object:
            self._road_network.get_roads()

        # Find the nearest road reference-line point and read its z
        best_z = 0.0
        best_dist = float("inf")
        query = np.array([xodr_x, xodr_y])

        for road in self._road_network.road_ids_to_object.values():
            ref_line = road.reference_line  # shape (N, 2)
            if len(ref_line) < 2:
                continue
            dists = np.linalg.norm(ref_line - query, axis=1)
            min_idx = int(np.argmin(dists))
            dist = float(dists[min_idx])
            if dist < best_dist:
                best_dist = dist
                z_coords = road.z_coordinates
                deltas = np.diff(ref_line, axis=0)
                seg_lengths = np.linalg.norm(deltas, axis=1)
                arc = np.zeros(len(ref_line))
                arc[1:] = np.cumsum(seg_lengths)
                best_z = float(np.interp(arc[min_idx], arc, z_coords))

        return ll2_z - best_z


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
