from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ExportConfig:
    input_path: Path
    output_path: Path
    report_path: Path
    log_path: Path
    road_thickness: float = 0.5
    surface_style: str = "solid"
    lanelet_side_overlap: float = 0.0
    wall_height: float = 2.0
    wall_thickness: float = 0.1
    marking_width: float = 0.05
    marking_thickness: float = 0.01
    marking_offset: float = 0.002
    marking_style: str = "prism"
    keep_intermediate: bool = False
    tmp_dir: Path | None = None
    origin_policy: str = "center"
    origin_shift_x: float = 0.0
    origin_shift_y: float = 0.0
    origin_shift_z: float = 0.0
    min_segment_length: float = 1e-3
    min_triangle_area: float = 1e-8

    @classmethod
    def from_args(cls, args) -> "ExportConfig":
        if args.lanelet_side_overlap < 0.0:
            raise ValueError("--lanelet-side-overlap must be non-negative.")
        if args.origin != "explicit" and any(
            shift != 0.0 for shift in (args.shift_x, args.shift_y, args.shift_z)
        ):
            raise ValueError(
                "--shift-x/--shift-y/--shift-z require --origin explicit."
            )
        tmp_dir = Path(args.tmp_dir).resolve() if args.tmp_dir else None
        report_path = Path(args.report).resolve()
        if args.log:
            log_path = Path(args.log).resolve()
        else:
            log_path = report_path.with_suffix(".log")
        return cls(
            input_path=Path(args.input).resolve(),
            output_path=Path(args.output).resolve(),
            report_path=report_path,
            log_path=log_path,
            road_thickness=args.road_thickness,
            surface_style=args.surface_style,
            lanelet_side_overlap=args.lanelet_side_overlap,
            wall_height=args.wall_height,
            wall_thickness=args.wall_thickness,
            marking_width=args.marking_width,
            marking_thickness=args.marking_thickness,
            marking_offset=args.marking_offset,
            marking_style=args.marking_style,
            keep_intermediate=args.keep_intermediate,
            tmp_dir=tmp_dir,
            origin_policy=args.origin,
            origin_shift_x=args.shift_x,
            origin_shift_y=args.shift_y,
            origin_shift_z=args.shift_z,
            min_segment_length=1e-3,
            min_triangle_area=1e-8,
        )
