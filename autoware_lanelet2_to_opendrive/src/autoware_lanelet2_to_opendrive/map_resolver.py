"""Utilities for using converted maps directly in CARLA scenario tests.

This module provides helpers that bridge the Lanelet2 → OpenDRIVE conversion
pipeline with a CARLA scenario testing framework such as
`autoware_carla_scenario <https://github.com/hakuturu583/autoware_carla_scenario>`_.  The central
entry-point is :func:`resolve_map_to_xodr`, which accepts either an existing
``.xodr`` file or a Lanelet2 ``.osm`` file and returns a path to a ready-to-use
OpenDRIVE file.  Lanelet2 maps are converted on first use and the result is
cached under ``/tmp`` so that subsequent calls are instant.

Cache layout::

    /tmp/autoware_lanelet2_to_opendrive_cache/
    └── <sha256_prefix>_<commit_prefix>/
        ├── map.xodr        # converted OpenDRIVE file
        └── cache_info.json # source SHA-256, xodr SHA-256, commit hash, paths

The cache is invalidated automatically when either the input file changes
(SHA-256 mismatch) or the converter library is updated (different git commit).

Example usage inside a scenario test::

    from pathlib import Path
    from autoware_lanelet2_to_opendrive import resolve_map_to_xodr
    from autoware_lanelet2_to_opendrive.conversion_config import (
        ConversionConfig,
        OriginSpec,
    )
    from autoware_carla_scenario import BaseScenario, EgoConfig, ScenarioQueue

    class MyScenario(BaseScenario):
        def setup(self, world):
            ...

        def is_done(self):
            return True

    config = ConversionConfig(origin=OriginSpec(mgrs_code="54SUE"))
    xodr_path = resolve_map_to_xodr(Path("map.osm"), config=config)

    queue = ScenarioQueue(xodr_path=xodr_path)
    queue.add(MyScenario(EgoConfig(spawn_index=0)))
    with queue:
        results = queue.run_all()
"""

from __future__ import annotations

import hashlib
import json
import logging
import subprocess
from pathlib import Path
from typing import Optional

import yaml

from .conversion_config import ConversionConfig
from .map_preflight import MapPreflightError, inspect_lanelet2_osm
from .map_projector_info import MAP_PROJECTOR_INFO_FILENAME

logger = logging.getLogger(__name__)

_CACHE_DIR = Path("/tmp/autoware_lanelet2_to_opendrive_cache")


def _reject_unsupported_cache_path_projector(lanelet2_path: Path) -> None:
    """Raise if a TransverseMercator ``map_projector_info.yaml`` sits next to *lanelet2_path*.

    The cache path bypasses the projector-resolution layer entirely -- it
    derives the origin only from ``mgrs_code``/``config.origin.lat``/
    ``config.origin.lon`` and never reads ``map_projector_info.yaml``, so a
    TransverseMercator map would silently be mis-projected as MGRS/lat-lon.
    Fail loudly instead and point users at the non-cache conversion flow,
    which does honor ``map_projector_info.yaml``. Reads the YAML directly
    (rather than importing :mod:`.projection_resolver`) to avoid pulling in
    ``lanelet2``/``autoware_lanelet2_extension_python`` for this cheap, early
    check; :mod:`.map_projector_info` only needs PyYAML.

    Args:
        lanelet2_path: Path to the Lanelet2 ``.osm``/``.bin`` file.

    Raises:
        RuntimeError: If a sibling ``map_projector_info.yaml`` declares
            ``projector_type: TransverseMercator``.
        ValueError: If the file's top-level YAML document is not a mapping.
    """
    info_path = lanelet2_path.parent / MAP_PROJECTOR_INFO_FILENAME
    if not info_path.is_file():
        return

    try:
        data = yaml.safe_load(info_path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError:
        # Malformed YAML is not this function's concern -- let the normal
        # conversion path surface the error.
        return
    if not isinstance(data, dict):
        raise ValueError(f"{info_path}: expected a YAML mapping at the top level")

    projector_type = str(data.get("projector_type", "")).strip()
    if projector_type == "TransverseMercator":
        raise RuntimeError(
            f"{info_path} declares projector_type: TransverseMercator, which "
            "the cached map-resolution path (resolve_map_to_xodr) does not "
            "support -- it does not read map_projector_info.yaml. Please use "
            "the non-cache conversion flow (preprocess_and_convert_with_hydra "
            "/ the `convert` CLI) instead, which resolves TransverseMercator "
            "origins correctly."
        )


def _sha256_of_file(path: Path) -> str:
    """Return the hex-encoded SHA-256 digest of *path*."""
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _get_converter_commit_hash() -> str:
    """Return the git HEAD commit hash of this package's repository.

    Walks up the directory tree from this file until a ``.git`` directory is
    found, then queries ``git rev-parse HEAD``.  Returns ``"unknown"`` when the
    commit cannot be determined (e.g. installed from a wheel).
    """
    search_dir = Path(__file__).resolve().parent
    for _ in range(10):
        if (search_dir / ".git").exists():
            break
        parent = search_dir.parent
        if parent == search_dir:
            return "unknown"
        search_dir = parent
    else:
        return "unknown"

    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=search_dir,
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return "unknown"


def _convert_lanelet2_to_xodr_cached(
    lanelet2_path: Path,
    config: Optional[ConversionConfig] = None,
    mgrs_code: Optional[str] = None,
    preflight: bool = True,
) -> Path:
    """Convert a Lanelet2 map to OpenDRIVE with ``/tmp``-based caching.

    Args:
        lanelet2_path: Path to the Lanelet2 ``.osm`` (or ``.bin``) file.
        config: Conversion configuration.  A default :class:`ConversionConfig`
                is used when *None*.
        mgrs_code: MGRS grid code for the coordinate origin.  When provided it
                   takes precedence over ``config.origin.mgrs_code``.
        preflight: Inspect the map's structure before loading it, and raise
                   :class:`~.map_preflight.MapPreflightError` with a report
                   naming what stops the load.  Pass *False* to get the raw
                   ``lanelet2.io.load`` exception back instead.  This mirrors
                   the CLI's ``preflight`` config key; it exists so that a
                   caller on this path has the same off switch, because a
                   false positive here would otherwise be unrecoverable
                   without pinning an older version of the package.

    Returns:
        Path to the cached ``.xodr`` file.

    Raises:
        FileNotFoundError: If *lanelet2_path* does not exist.
        ValueError: If the map origin cannot be determined from *config* or
                    *mgrs_code*.
        RuntimeError: If a sibling ``map_projector_info.yaml`` declares
                    ``projector_type: TransverseMercator``; the cache path
                    does not support it.
    """
    if not lanelet2_path.exists():
        raise FileNotFoundError(f"Lanelet2 map not found: {lanelet2_path}")

    _reject_unsupported_cache_path_projector(lanelet2_path)

    if config is None:
        config = ConversionConfig()

    # ------------------------------------------------------------------
    # Build cache key and check for a hit
    # (origin is deferred until after the hit check – not needed on hits)
    # ------------------------------------------------------------------
    source_sha256 = _sha256_of_file(lanelet2_path)
    converter_commit = _get_converter_commit_hash()
    cache_key = f"{source_sha256[:16]}_{converter_commit[:12]}"

    cache_dir = _CACHE_DIR / cache_key
    cached_xodr = cache_dir / "map.xodr"
    cache_info_file = cache_dir / "cache_info.json"

    if cached_xodr.exists() and cache_info_file.exists():
        logger.info("Cache hit – reusing converted OpenDRIVE map: %s", cached_xodr)
        return cached_xodr

    # ------------------------------------------------------------------
    # Determine coordinate origin (only needed on cache miss)
    # ------------------------------------------------------------------
    from .projection import latlon_to_lanelet2_origin, mgrs_to_lanelet2_origin

    # Merge the separate mgrs_code argument (legacy API) into config.origin so
    # that the converter has a single, consistent source of truth.
    effective_mgrs = mgrs_code or config.origin.mgrs_code
    if effective_mgrs is not None:
        config = config.with_mgrs_code(effective_mgrs)

    if effective_mgrs is not None:
        origin = mgrs_to_lanelet2_origin(effective_mgrs)
    elif config.origin.lat is not None and config.origin.lon is not None:
        origin = latlon_to_lanelet2_origin(config.origin.lat, config.origin.lon)
    else:
        raise ValueError(
            "Cannot convert Lanelet2 map: no origin specified. "
            "Provide mgrs_code or set lat/lon in ConversionConfig.origin."
        )

    # ------------------------------------------------------------------
    # Load Lanelet2 map and convert
    # ------------------------------------------------------------------
    logger.info("Converting Lanelet2 map to OpenDRIVE: %s", lanelet2_path)

    # Import autoware extensions before loading maps (registers custom types)
    from autoware_lanelet2_extension_python.projection import MGRSProjector  # noqa: F401
    import lanelet2

    from .main import convert_lanelet2_to_opendrive
    from .opendrive.opendrive import save_opendrive_to_file
    from .road_lanelet_geo_mapping import (
        GeoRoadLaneletMapping,
        save_mapping_json,
    )

    projector = MGRSProjector(origin)

    # Same pre-flight the CLI runs (see main.preprocess_and_convert_with_hydra):
    # name the structural problems that stop the load, instead of letting the
    # parser's flat error wall out. Diagnostic only -- every blocking finding
    # is a constraint lanelet2.io.load enforces anyway.
    #
    # `inspected` is tested before `ok` on purpose: a file lxml cannot parse
    # must be reported as uninspected and let through, never refused. The
    # order makes the guard structural rather than a consequence of
    # `inspect_lanelet2_osm` happening to return early with no findings.
    if preflight and lanelet2_path.suffix == ".osm":
        report = inspect_lanelet2_osm(lanelet2_path)
        if not report.inspected:
            logger.warning("Pre-flight could not parse the map: %s", report.xml_error)
        elif not report.ok:
            raise MapPreflightError(report)
        elif report.findings:
            # Non-blocking only, or the branch above would have fired. The
            # CLI prints these; this path logged nothing, which made the same
            # inspection half as useful depending on how it was reached.
            logger.warning("Pre-flight found non-fatal problems: %s", report.format())

    # `str(...)`: the Boost.Python binding's signature takes a std::string
    # and rejects a PosixPath outright, so this path could not load any .osm
    # map at all. `main.py` and `analyze_xodr.py` both convert; this did not.
    lanelet_map = lanelet2.io.load(str(lanelet2_path), projector)

    # mgrs_code is already in config.origin.mgrs_code (merged above).
    opendrive, _, lanelet_to_road_and_lane, _, _ = convert_lanelet2_to_opendrive(
        lanelet_map, config
    )

    # ------------------------------------------------------------------
    # Persist to cache
    # ------------------------------------------------------------------
    cache_dir.mkdir(parents=True, exist_ok=True)
    save_opendrive_to_file(opendrive, cached_xodr)

    # Save mapping JSON next to the cached XODR
    xodr_sha256 = _sha256_of_file(cached_xodr)
    osm_sha256 = _sha256_of_file(lanelet2_path)
    conv_mapping = GeoRoadLaneletMapping(
        xodr_sha256=xodr_sha256,
        osm_sha256=osm_sha256,
        lanelet_to_road_and_lane=lanelet_to_road_and_lane,
    )
    save_mapping_json(conv_mapping, cached_xodr)

    cache_info: dict = {
        "source_sha256": source_sha256,
        "converter_commit": converter_commit,
        "source_path": str(lanelet2_path),
    }
    with open(cache_info_file, "w") as fh:
        json.dump(cache_info, fh, indent=2)

    logger.info("Converted map saved to cache: %s", cached_xodr)
    return cached_xodr


def resolve_map_to_xodr(
    map_path: Path | str,
    config: Optional[ConversionConfig] = None,
    mgrs_code: Optional[str] = None,
    preflight: bool = True,
) -> Path:
    """Resolve a map file to an OpenDRIVE (``.xodr``) path.

    Inspects the file extension to decide how to handle the input:

    * ``.xodr`` – returned as-is; no conversion is performed.
    * Any other extension (e.g. ``.osm``, ``.bin``) – treated as a Lanelet2
      map.  The map is converted to OpenDRIVE and the result is cached in
      ``/tmp`` so that repeated calls with the same input are instant.

    Args:
        map_path: Path to the map file (``.xodr`` or Lanelet2 format).
        config: Conversion configuration for Lanelet2 → OpenDRIVE conversion.
                Required (or *mgrs_code* must be set) when the input is a
                Lanelet2 map.
        mgrs_code: MGRS grid code for the coordinate origin.  Overrides the
                   value in ``config.origin`` if provided.
        preflight: Inspect the map's structure before loading it (default).
                   Pass *False* to restore the previous behaviour, where
                   ``lanelet2.io.load``'s own exception comes out unexamined.

    Returns:
        Absolute :class:`~pathlib.Path` to a ``.xodr`` file.

    Raises:
        FileNotFoundError: If *map_path* does not exist (Lanelet2 path only).
        ValueError: If the map origin cannot be determined (Lanelet2 path only).

    Example::

        from autoware_lanelet2_to_opendrive import resolve_map_to_xodr
        from autoware_lanelet2_to_opendrive.conversion_config import (
            ConversionConfig, OriginSpec,
        )

        # Direct .xodr – returned unchanged
        xodr = resolve_map_to_xodr("map.xodr")

        # Lanelet2 .osm – converted and cached automatically
        config = ConversionConfig(origin=OriginSpec(mgrs_code="54SUE"))
        xodr = resolve_map_to_xodr("map.osm", config=config)
    """
    map_path = Path(map_path)
    if map_path.suffix == ".xodr":
        return map_path
    return _convert_lanelet2_to_xodr_cached(map_path, config, mgrs_code, preflight)
