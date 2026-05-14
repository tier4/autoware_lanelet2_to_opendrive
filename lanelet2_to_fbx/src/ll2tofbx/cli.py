from __future__ import annotations

import argparse
import json
import logging
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from .config import ExportConfig
from .godot_extract import extract_feature_set
from .lanelet_loader import load_lanelet_map
from .layer_names import LAYER_NAMES, ROAD_SURFACE_LAYER_NAMES
from .mesh_types import Vec3
from .obj_writer import write_obj_bundle
from .osm_index import load_osm_index
from .solid_builder import (
    build_marking_layer,
    build_surface_layer,
    build_wall_layer,
    compute_origin_shift,
)
from .validate import validate_layers


LAYER_ORDER = list(LAYER_NAMES)


class ExportError(RuntimeError):
    """Raised when export cannot complete."""


class ValidationFailure(ExportError):
    """Raised when generated geometry fails validation."""

    def __init__(self, validation_report, partial_result: dict[str, object] | None = None) -> None:
        self.validation_report = validation_report
        self.partial_result = partial_result or {}
        super().__init__("Validation failed.")


LOGGER = logging.getLogger("ll2tofbx")


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.command != "export":
        parser.error("A subcommand is required.")

    config = ExportConfig.from_args(args)
    _configure_logging(config.log_path)
    report = {
        "input_path": str(config.input_path),
        "output_path": str(config.output_path),
        "log_path": str(config.log_path),
        "origin_mode": config.origin_policy,
        "origin_shift_xyz": [0.0, 0.0, 0.0],
        "layer_stats": {},
        "filtered_source_counts": {},
        "validation": {
            "errors": [],
            "warnings": [],
            "degenerate_triangles": 0,
            "open_edges": 0,
            "non_finite_vertices": 0,
            "inward_faces": 0,
        },
        "blender_command": "",
        "success": False,
    }

    temp_dir: Path | None = None
    try:
        LOGGER.info("Starting export")
        LOGGER.info("Input: %s", config.input_path)
        LOGGER.info("Output: %s", config.output_path)
        LOGGER.info("Report: %s", config.report_path)
        LOGGER.info("Log: %s", config.log_path)
        temp_dir = Path(
            tempfile.mkdtemp(
                prefix="ll2tofbx_",
                dir=str(config.tmp_dir) if config.tmp_dir else None,
            )
        )
        LOGGER.info("Intermediate directory: %s", temp_dir)
        result = run_export(config, temp_dir)
        report.update(result)
        report["success"] = True
        LOGGER.info("Export completed successfully")
    except ValidationFailure as exc:
        report.update(exc.partial_result)
        report["validation"] = exc.validation_report.as_dict()
        LOGGER.error("Validation failed")
        for error in exc.validation_report.errors:
            LOGGER.error("%s", error)
    except Exception as exc:
        report["validation"]["errors"].append(str(exc))
        LOGGER.exception("Export failed: %s", exc)
    finally:
        config.report_path.parent.mkdir(parents=True, exist_ok=True)
        with config.report_path.open("w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
            handle.write("\n")
        LOGGER.info("Wrote report to %s", config.report_path)
        if temp_dir and temp_dir.exists() and not config.keep_intermediate:
            shutil.rmtree(temp_dir, ignore_errors=True)
            LOGGER.info("Removed intermediate directory: %s", temp_dir)

    return 0 if report["success"] else 1


def run_export(config: ExportConfig, temp_dir: Path) -> dict[str, object]:
    root_name = config.input_path.stem
    filtered_counts: dict[str, int] = {}

    osm_index = load_osm_index(config.input_path)
    LOGGER.info("Indexed OSM nodes: %d", len(osm_index.nodes))
    loaded_map = load_lanelet_map(config.input_path, osm_index)
    LOGGER.info("Loaded lanelet map with %d load warnings", len(loaded_map.load_errors))
    feature_set = extract_feature_set(loaded_map.lanelet_map, loaded_map.coordinate_resolver)
    LOGGER.info(
        "Extracted features: lanelets=%d intersection=%d hatched=%d parking=%d borders=%d shoulders=%d markings=%d",
        len(feature_set.lanelet_roads),
        len(feature_set.intersection_areas),
        len(feature_set.hatched_areas),
        len(feature_set.parking_lots),
        len(feature_set.road_borders),
        len(feature_set.shoulders),
        len(feature_set.road_markings),
    )
    if config.marking_style == "flat":
        LOGGER.info(
            "Using flat road_marking mode; --marking-thickness is ignored and --marking-offset controls lift."
        )
    if config.surface_style == "flat":
        LOGGER.info(
            "Using flat surface mode; --road-thickness is ignored and ground layers are emitted as double-sided surfaces."
        )

    layers = [
        build_surface_layer(
            "lanelet_road",
            feature_set.lanelet_roads,
            config.road_thickness,
            config.min_segment_length,
            config.min_triangle_area,
            filtered_counts,
            LOGGER,
            surface_style=config.surface_style,
        ),
    ]
    if config.lanelet_side_overlap > 0.0:
        layers.append(
            build_surface_layer(
                "road_extention",
                feature_set.lanelet_roads,
                config.road_thickness,
                config.min_segment_length,
                config.min_triangle_area,
                filtered_counts,
                LOGGER,
                side_overlap=config.lanelet_side_overlap,
                surface_style=config.surface_style,
            )
        )
    layers.extend(
        [
            build_surface_layer(
            "intersection_area",
            feature_set.intersection_areas,
            config.road_thickness,
            config.min_segment_length,
            config.min_triangle_area,
            filtered_counts,
            LOGGER,
            surface_style=config.surface_style,
        ),
        build_surface_layer(
            "hatched_area",
            feature_set.hatched_areas,
            config.road_thickness,
            config.min_segment_length,
            config.min_triangle_area,
            filtered_counts,
            LOGGER,
            surface_style=config.surface_style,
        ),
        build_surface_layer(
            "parking_lot",
            feature_set.parking_lots,
            config.road_thickness,
            config.min_segment_length,
            config.min_triangle_area,
            filtered_counts,
            LOGGER,
            surface_style=config.surface_style,
        ),
        build_surface_layer(
            "shoulder",
            feature_set.shoulders,
            config.road_thickness,
            config.min_segment_length,
            config.min_triangle_area,
            filtered_counts,
            LOGGER,
            surface_style=config.surface_style,
        ),
        build_wall_layer(
            "road_border_wall",
            feature_set.road_borders,
            config.wall_height,
            config.wall_thickness,
            config.min_segment_length,
            config.min_triangle_area,
            filtered_counts,
            LOGGER,
        ),
        build_marking_layer(
            "road_marking",
            feature_set.road_markings,
            config.marking_width,
            config.marking_thickness,
            config.marking_offset,
            config.marking_style,
            config.min_segment_length,
            config.min_triangle_area,
            filtered_counts,
            LOGGER,
        ),
        ]
    )
    for layer in layers:
        LOGGER.info(
            "Built layer %s: features=%d vertices=%d triangles=%d",
            layer.name,
            layer.source_feature_count,
            len(layer.mesh.vertices),
            len(layer.mesh.faces),
        )

    road_top_vertices = [
        vertex
        for layer in layers
        if layer.name in ROAD_SURFACE_LAYER_NAMES
        for vertex in layer.top_vertices
    ]
    origin_shift = _resolve_origin_shift(config, road_top_vertices, layers)
    LOGGER.info("Origin shift: %s", origin_shift.as_list())
    shifted_layers = [layer.translated(origin_shift) for layer in layers]

    layer_stats = {layer.name: _layer_stats(layer) for layer in shifted_layers}
    partial_result = {
        "origin_shift_xyz": origin_shift.as_list(),
        "layer_stats": layer_stats,
        "filtered_source_counts": filtered_counts,
        "load_errors": loaded_map.load_errors,
        "intermediate_dir": str(temp_dir),
    }

    validation = validate_layers(shifted_layers, config.min_triangle_area)
    if validation.errors:
        raise ValidationFailure(validation, partial_result)

    obj_path, _ = write_obj_bundle(root_name, temp_dir, shifted_layers)
    blender_command = _run_blender_export(root_name, obj_path, config.output_path)
    return {
        **partial_result,
        "validation": validation.as_dict(),
        "blender_command": blender_command,
    }


def _resolve_origin_shift(config: ExportConfig, road_top_vertices, layers):
    if config.origin_policy == "center":
        return compute_origin_shift(road_top_vertices, layers)
    if config.origin_policy == "explicit":
        return Vec3(
            config.origin_shift_x,
            config.origin_shift_y,
            config.origin_shift_z,
        )
    raise ExportError(f"Unsupported origin policy: {config.origin_policy}")


def _run_blender_export(root_name: str, obj_path: Path, output_path: Path) -> str:
    blender_bin = os.environ.get("BLENDER_BIN", "blender")
    if shutil.which(blender_bin) is None:
        raise ExportError(
            f"Blender executable '{blender_bin}' was not found. Set BLENDER_BIN or install Blender."
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    script_path = _resolve_blender_script_path()
    command = [
        blender_bin,
        "--background",
        "--factory-startup",
        "--python",
        str(script_path),
        "--",
        "--input-obj",
        str(obj_path),
        "--output-fbx",
        str(output_path),
        "--root-name",
        root_name,
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip() or "unknown Blender export failure"
        raise ExportError(f"Blender export failed: {message}")
    if not output_path.exists():
        stdout = result.stdout.strip()
        stderr = result.stderr.strip()
        message = f"Blender finished without writing '{output_path}'."
        if stdout:
            message += f" stdout: {stdout}"
        if stderr:
            message += f" stderr: {stderr}"
        raise ExportError(message)
    return shlex.join(command)


def _resolve_blender_script_path() -> Path:
    package_script = Path(__file__).resolve().with_name("blender_obj_to_fbx.py")
    if package_script.exists():
        return package_script

    raise ExportError("Could not locate blender_obj_to_fbx.py.")


def _layer_stats(layer) -> dict[str, object]:
    aabb = layer.mesh.aabb()
    return {
        "source_feature_count": layer.source_feature_count,
        "vertex_count": len(layer.mesh.vertices),
        "triangle_count": len(layer.mesh.faces),
        "aabb_min": aabb[0].as_list() if aabb else None,
        "aabb_max": aabb[1].as_list() if aabb else None,
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ll2tofbx")
    subparsers = parser.add_subparsers(dest="command")

    export = subparsers.add_parser("export", help="Convert a Lanelet2 OSM map to FBX.")
    export.add_argument("--input", required=True, help="Input Lanelet2 OSM file.")
    export.add_argument("--output", required=True, help="Output FBX path.")
    export.add_argument("--report", required=True, help="Output JSON report path.")
    export.add_argument("--log", default=None, help="Output log path. Defaults to report path with .log.")
    export.add_argument("--road-thickness", type=float, default=0.5)
    export.add_argument(
        "--surface-style",
        choices=["solid", "flat"],
        default="solid",
        help="Ground surface geometry mode. 'flat' keeps road-like ground layers thickness-free and double-sided.",
    )
    export.add_argument(
        "--lanelet-side-overlap",
        type=float,
        default=0.0,
        help="Expand lanelet road surfaces outward on both sides before triangulation.",
    )
    export.add_argument("--wall-height", type=float, default=2.0)
    export.add_argument("--wall-thickness", type=float, default=0.1)
    export.add_argument("--marking-width", type=float, default=0.05)
    export.add_argument(
        "--marking-style",
        choices=["prism", "flat"],
        default="prism",
        help="Road marking geometry mode. 'flat' keeps markings nearly flush using --marking-offset only.",
    )
    export.add_argument("--marking-thickness", type=float, default=0.01)
    export.add_argument("--marking-offset", type=float, default=0.002)
    export.add_argument("--keep-intermediate", action="store_true")
    export.add_argument("--tmp-dir", default=None)
    export.add_argument(
        "--origin",
        choices=["center", "explicit"],
        default="center",
        help="Origin policy. Use 'explicit' to subtract a caller-specified shift.",
    )
    export.add_argument(
        "--shift-x",
        type=float,
        default=0.0,
        help="X shift to subtract when --origin explicit is selected.",
    )
    export.add_argument(
        "--shift-y",
        type=float,
        default=0.0,
        help="Y shift to subtract when --origin explicit is selected.",
    )
    export.add_argument(
        "--shift-z",
        type=float,
        default=0.0,
        help="Z shift to subtract when --origin explicit is selected.",
    )
    return parser


def _configure_logging(log_path: Path) -> None:
    LOGGER.handlers.clear()
    LOGGER.setLevel(logging.INFO)
    LOGGER.propagate = False
    log_path.parent.mkdir(parents=True, exist_ok=True)
    formatter = logging.Formatter(
        fmt="%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    file_handler = logging.FileHandler(log_path, mode="w", encoding="utf-8")
    file_handler.setFormatter(formatter)
    stream_handler = logging.StreamHandler(sys.stderr)
    stream_handler.setFormatter(formatter)
    LOGGER.addHandler(file_handler)
    LOGGER.addHandler(stream_handler)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
