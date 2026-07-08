"""Command-line interface for the OpenSCENARIO DSL frontend.

Installed as the ``osc-transpile`` console script::

    osc-transpile scenario.osc                    # print Python to stdout
    osc-transpile scenario.osc -o scenario.py     # write a single Python module
    osc-transpile scenario.osc --package -o out/  # generate a scenario package
    osc-transpile scenario.osc --check            # syntax-check only
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .errors import OscError
from .parser import parse_osc_file
from .transpiler import transpile_file, transpile_to_package


def _build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="osc-transpile",
        description=(
            "Transpile an OpenSCENARIO DSL (.osc) file into a readable "
            "autoware_carla_scenario Python scenario or scenario package."
        ),
    )
    parser.add_argument("source", help="Path to the .osc source file")
    parser.add_argument(
        "-o",
        "--output",
        help=(
            "Output location: a .py file (default mode) or, with --package, the "
            "directory to create the package in (default: print / current dir)"
        ),
    )
    parser.add_argument(
        "-p",
        "--package",
        action="store_true",
        help="Generate an installable scenario package instead of a single module",
    )
    parser.add_argument(
        "--name",
        help="Package name (with --package; default: <scenario>_package)",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="Overwrite existing files (with --package)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Only parse and syntax-check the source; produce no output",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Entry point for the ``osc-transpile`` console script."""
    args = _build_argparser().parse_args(argv)
    try:
        if args.check:
            parse_osc_file(args.source)
            print(f"OK: {args.source} parsed without errors", file=sys.stderr)
            return 0

        if args.package:
            root = transpile_to_package(
                args.source,
                output_dir=args.output or ".",
                package_name=args.name,
                force=args.force,
            )
            print(f"Created scenario package at {root}", file=sys.stderr)
            print("\nNext steps:", file=sys.stderr)
            print(f"  uv pip install -e {root}", file=sys.stderr)
            print(
                "  uv run scenario scenario=<name>/default map=nishishinjuku",
                file=sys.stderr,
            )
            return 0

        code = transpile_file(args.source)
        if args.output:
            Path(args.output).write_text(code, encoding="utf-8")
            print(f"Wrote {args.output}", file=sys.stderr)
        else:
            sys.stdout.write(code)
        return 0
    except OscError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except (FileNotFoundError, FileExistsError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
