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
import tempfile
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
        xodr_path: Optional[Path] = None,
        lanelet2_path: Path = None,  # type: ignore[assignment]
        carla_world: Any = None,
    ) -> None:
        """Load the OpenDRIVE road network and the Lanelet2 map.

        Parameters
        ----------
        xodr_path:
            Path to the OpenDRIVE (.xodr) file.  When ``None``, the OpenDRIVE is
            sourced from the live CARLA world instead (``carla_world`` is then
            required) -- the scenario needs to ship only the Lanelet2 map, since
            CARLA already holds the matching OpenDRIVE for its loaded level.
        lanelet2_path:
            Path to the Lanelet2 map file (.osm or .xml).
        carla_world:
            CARLA ``carla.World`` instance.  Required when *xodr_path* is ``None``
            (its ``get_map().to_opendrive()`` provides the OpenDRIVE).  When
            provided, the vertical offset (z_offset) is also computed by averaging
            the difference between Lanelet2 elevation and CARLA spawn-point
            elevation; when ``None``, a single-point XODR fallback is used.

        Raises
        ------
        RuntimeError
            If already initialized.
        ValueError
            If neither *xodr_path* nor *carla_world* is given.
        FileNotFoundError
            If a provided map file does not exist.
        """
        if self._lanelet_map is not None or self._road_network is not None:
            raise RuntimeError(
                "MapManager is already initialized. "
                "Call MapManager.reset() before re-initializing (testing only)."
            )

        if not lanelet2_path.exists():
            raise FileNotFoundError(f"Lanelet2 file not found: {lanelet2_path}")

        # Resolve the OpenDRIVE source, most specific first:
        #   1. a supplied .xodr file;
        #   2. the live CARLA world's OpenDRIVE (a scenario then ships only the
        #      Lanelet2 map);
        #   3. neither -> Lanelet2-only: derive the projection origin from the
        #      Lanelet2 map itself.  OpenDRIVE-based features (road network,
        #      lanelet<->road mapping, OpenDRIVE conversions) are unavailable and
        #      raise if used.
        # A CARLA-sourced OpenDRIVE is written to a temp file because pyxodr reads
        # a path; it is removed once parsing is done.
        temp_xodr: Optional[Path] = None
        xodr_content: Optional[str] = None
        if xodr_path is not None:
            if not xodr_path.exists():
                raise FileNotFoundError(f"OpenDRIVE file not found: {xodr_path}")
            xodr_content = xodr_path.read_text(encoding="utf-8")
        elif carla_world is not None:
            candidate = carla_world.get_map().to_opendrive()
            if _has_geo_reference(candidate):
                xodr_content = candidate
                handle, name = tempfile.mkstemp(suffix=".xodr")
                temp_xodr = Path(name)
                with open(handle, "w", encoding="utf-8") as tmp:
                    tmp.write(xodr_content)
                xodr_path = temp_xodr

        try:
            if xodr_content is not None:
                assert xodr_path is not None  # noqa: S101 - set whenever content is
                self._load_with_opendrive(
                    xodr_path, xodr_content, lanelet2_path, carla_world
                )
            else:
                self._load_lanelet2_only(lanelet2_path, carla_world)
        finally:
            if temp_xodr is not None:
                temp_xodr.unlink(missing_ok=True)

    def _load_with_opendrive(
        self,
        xodr_path: Path,
        xodr_content: str,
        lanelet2_path: Path,
        carla_world: Any,
    ) -> None:
        """Load the Lanelet2 map and the OpenDRIVE road network (full features)."""
        # Parse geoReference from XODR to get the UTM origin
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

    def _load_lanelet2_only(self, lanelet2_path: Path, carla_world: Any) -> None:
        """Load only the Lanelet2 map, deriving the projection from the map itself.

        No OpenDRIVE is available, so :attr:`road_network` and the
        lanelet<->road mapping stay unset and OpenDRIVE conversions raise.  The
        Lanelet2 map is self-describing (its nodes carry ``lat``/``lon`` and
        MGRS ``local_x``/``local_y``), which is all that
        :func:`~autoware_carla_scenario.coordinate.transform.lanelet2_to_map` and
        the CARLA<->Lanelet2 conversions need.
        """
        lat, lon, alt = _parse_lanelet2_origin(lanelet2_path)
        self._geo_origin = (lat, lon, alt)

        origin = lanelet2.io.Origin(lat, lon)
        projector = MGRSProjector(origin)
        self._lanelet_map = lanelet2.io.load(str(lanelet2_path), projector)

        fwd = projector.forward(lanelet2.core.GPSPoint(lat, lon, alt))
        self._mgrs_offset = (fwd.x, fwd.y)

        # No OpenDRIVE: leave the road-based state empty (accessors raise).
        self._road_network = None
        self._road_lanelet_mapping = None
        self._carla_map = None
        # Lanelet2 z is absolute; without an XODR reference there is nothing to
        # offset against, so CARLA<->Lanelet2 z is treated as aligned.
        self._z_offset = 0.0

        logger.info(
            "MapManager loaded Lanelet2-only (no OpenDRIVE); road-network / "
            "OpenDRIVE conversions are unavailable."
        )

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
    def has_opendrive(self) -> bool:
        """Whether an OpenDRIVE road network is available.

        ``False`` for a lanelet2-only map, where OpenDRIVE-based conversions
        (:func:`~.transform.to_opendrive`, :attr:`road_network`) are
        unavailable and callers must work in Lanelet2/map coordinates instead.
        """
        return self._road_network is not None

    @property
    def road_network(self) -> RoadNetwork:
        """The loaded pyxodr.RoadNetwork instance (roads are pre-loaded)."""
        if self._road_network is None:
            if self._lanelet_map is not None:
                raise RuntimeError(
                    "OpenDRIVE is unavailable: this map was loaded Lanelet2-only "
                    "(no .xodr and no CARLA OpenDRIVE). OpenDRIVE-based conditions "
                    "and conversions (e.g. EntityLanePositionCondition, "
                    "to_opendrive) are not supported for this map; write the "
                    "scenario in Lanelet2/map coordinates (e.g. EntityInAreaCondition)."
                )
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


def _has_geo_reference(xodr_content: str) -> bool:
    """Return whether *xodr_content* carries a parseable geoReference origin.

    Some CARLA levels return an OpenDRIVE without a usable ``<geoReference>``; in
    that case the map must fall back to a Lanelet2-only load.
    """
    try:
        _parse_geo_reference(xodr_content)
        return True
    except ValueError:
        return False


def _parse_lanelet2_origin(lanelet2_path: Path) -> tuple[float, float, float]:
    """Return an ``(lat, lon, alt)`` origin from the Lanelet2 map's first node.

    Lanelet2 OSM nodes carry ``lat``/``lon`` attributes; any node in the map's
    MGRS grid yields the same local coordinates, so the first one is a fine
    origin for the ``MGRSProjector`` (matching how the map's ``local_x``/
    ``local_y`` were generated).

    Raises:
        ValueError: If no ``<node>`` with ``lat``/``lon`` is found.
    """
    with open(lanelet2_path, encoding="utf-8") as osm:
        for line in osm:
            if "<node" not in line:
                continue
            lat_match = re.search(r'lat=["\']([-\d.]+)["\']', line)
            lon_match = re.search(r'lon=["\']([-\d.]+)["\']', line)
            if lat_match and lon_match:
                return float(lat_match.group(1)), float(lon_match.group(1)), 0.0
    raise ValueError(f"No <node> with lat/lon found in Lanelet2 map: {lanelet2_path}")
