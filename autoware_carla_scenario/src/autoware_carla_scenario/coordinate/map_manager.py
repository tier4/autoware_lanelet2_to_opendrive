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

import logging
import re
from pathlib import Path
from typing import Any, ClassVar, Optional

# autoware_lanelet2_extension_python must be imported before lanelet2 to register
# Autoware-specific regulatory elements (road_marking, detection_area, etc.)
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2.core
import lanelet2.io
from pyxodr.road_objects.network import RoadNetwork

from .road_lanelet_mapping import RoadLaneletMapping

logger = logging.getLogger(__name__)


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
    _road_lanelet_mapping: Optional[RoadLaneletMapping]
    _carla_map: Optional[Any]

    def __new__(cls) -> "MapManager":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._lanelet_map = None
            cls._instance._road_network = None
            cls._instance._geo_origin = None
            cls._instance._mgrs_offset = None
            cls._instance._z_offset = None
            cls._instance._road_lanelet_mapping = None
            cls._instance._carla_map = None
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

    def initialize(
        self,
        xodr_path: Path,
        lanelet2_path: Path,
        carla_world: Any = None,
    ) -> None:
        """Load both map files.

        Parameters
        ----------
        xodr_path:
            Path to the OpenDRIVE (.xodr) file.
        lanelet2_path:
            Path to the Lanelet2 map file (.osm or .xml).
        carla_world:
            Optional CARLA ``carla.World`` instance.  When provided, the
            vertical offset (z_offset) is computed by averaging the
            difference between Lanelet2 elevation and CARLA spawn-point
            elevation across all map spawn points.  When ``None``, a
            single-point fallback using the XODR reference line is used.

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
        self._z_offset = self._compute_z_offset(carla_world)

        # Build lanelet -> (road_id, lane_id) mapping for direct conversion.
        try:
            from .road_lanelet_mapping import load_or_build_mapping  # noqa: PLC0415
            from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
                parse_roads_from_xodr,
            )

            parsed_roads = parse_roads_from_xodr(xodr_path)
            self._road_lanelet_mapping = load_or_build_mapping(
                xodr_path=xodr_path,
                osm_path=lanelet2_path,
                lanelet_map=self.lanelet_map,
                roads=parsed_roads,
                mgrs_offset=self._mgrs_offset,
            )
        except Exception:
            logger.warning(
                "Failed to build lanelet-to-road mapping; "
                "direct Lanelet2 -> OpenDRIVE conversion unavailable",
                exc_info=True,
            )
            self._road_lanelet_mapping = None

        # Build carla.Map for waypoint-based road/lane lookups (optional).
        self._build_carla_map(xodr_content, xodr_path.stem, carla_world)

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
    def road_lanelet_mapping(self) -> Optional[RoadLaneletMapping]:
        """The lanelet -> (road_id, lane_id) mapping, or ``None`` if unavailable."""
        return self._road_lanelet_mapping

    @property
    def carla_map(self) -> Optional[Any]:
        """The ``carla.Map`` instance, or ``None`` if unavailable.

        Used by :func:`~.transform._carla_to_opendrive_via_waypoint` to look
        up the exact road/lane for a given world location.
        """
        return self._carla_map

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

    def _build_carla_map(
        self,
        xodr_content: str,
        map_name: str,
        carla_world: Any = None,
    ) -> None:
        """Build and store a ``carla.Map`` for waypoint lookups.

        When *carla_world* is provided its map is used directly (matches the
        running simulation).  When no world is available (e.g. unit tests),
        ``_carla_map`` stays ``None`` and the brute-force fallback is used
        instead.

        Failures are silently caught so that the rest of MapManager
        initialisation is unaffected.
        """
        if carla_world is None:
            return

        try:
            self._carla_map = carla_world.get_map()
        except Exception:
            logger.debug(
                "Could not build carla.Map; waypoint-based lookup disabled",
                exc_info=True,
            )
            self._carla_map = None

    def _compute_z_offset(self, carla_world: Any = None) -> float:
        """Compute ``lanelet2_z − carla_z``.

        When *carla_world* is provided, every CARLA spawn point is projected
        onto the nearest Lanelet2 centerline and the per-point offset
        ``(lanelet2_z − carla_spawn_z)`` is averaged.  This yields a more
        accurate result than single-point sampling because it smooths out
        local interpolation noise.

        When *carla_world* is ``None`` (e.g. in unit tests without a CARLA
        connection) the legacy single-point fallback is used.
        """
        if carla_world is not None:
            result = self._z_offset_from_spawn_points(carla_world)
            if result is not None:
                return result
        return self._z_offset_from_reference_line()

    # -- spawn-point based (preferred) ------------------------------------

    def _z_offset_from_spawn_points(self, carla_world: Any) -> Optional[float]:
        """Average ``lanelet2_z − carla_z`` over all CARLA spawn points.

        Returns ``None`` when no usable spawn points are found so the caller
        can fall back to the reference-line method.
        """
        import lanelet2.core
        import lanelet2.geometry
        import numpy as np

        assert self._lanelet_map is not None
        assert self._mgrs_offset is not None

        spawn_points = carla_world.get_map().get_spawn_points()
        if not spawn_points:
            return None

        offset_x, offset_y = self._mgrs_offset
        offsets: list[float] = []

        for sp in spawn_points:
            # CARLA world → Lanelet2 MGRS (x, y)
            mgrs_x = sp.location.x + offset_x
            mgrs_y = -sp.location.y + offset_y

            query = lanelet2.core.BasicPoint2d(mgrs_x, mgrs_y)
            results = lanelet2.geometry.findNearest(
                self._lanelet_map.laneletLayer, query, 1
            )
            if not results:
                continue

            dist_to_lanelet = results[0][0]
            # Skip spawn points too far from any lanelet (off-road areas)
            if dist_to_lanelet > 10.0:
                continue

            lanelet = results[0][1]

            # Nearest centerline point → its z
            best_d2 = float("inf")
            ll2_z = 0.0
            for pt in lanelet.centerline:
                d2 = (pt.x - mgrs_x) ** 2 + (pt.y - mgrs_y) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    ll2_z = pt.z

            offsets.append(ll2_z - sp.location.z)

        if not offsets:
            return None

        return float(np.mean(offsets))

    # -- single-point fallback (for tests without CARLA) ------------------

    def _z_offset_from_reference_line(self) -> float:
        """Compute ``lanelet2_z − xodr_z`` by sampling one reference point.

        Takes the first lanelet centerline point, finds the nearest XODR road
        reference-line point at the same (x, y), and returns the difference
        in their z coordinates.
        """
        import numpy as np

        assert self._lanelet_map is not None
        for lanelet in self._lanelet_map.laneletLayer:
            ref_pt = lanelet.centerline[0]
            break
        else:
            return 0.0

        ll2_z = ref_pt.z
        assert self._mgrs_offset is not None
        xodr_x = ref_pt.x - self._mgrs_offset[0]
        xodr_y = ref_pt.y - self._mgrs_offset[1]

        assert self._road_network is not None
        if not self._road_network.road_ids_to_object:
            self._road_network.get_roads()

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
