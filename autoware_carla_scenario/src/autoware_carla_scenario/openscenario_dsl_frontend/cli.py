"""Command-line interface for the OpenSCENARIO DSL frontend.

Installed as the ``osc-transpile`` console script::

    osc-transpile scenario.osc                # print Python to stdout
    osc-transpile scenario.osc -o scenario.py # write Python to a file
    osc-transpile scenario.osc --check        # syntax-check only
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .errors import OscError
from .parser import parse_osc_file
from .transpiler import transpile_file


def _build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="osc-transpile",
        description=(
            "Transpile an OpenSCENARIO DSL (.osc) file into a readable "
            "autoware_carla_scenario Python scenario."
        ),
    )
    parser.add_argument("source", help="Path to the .osc source file")
    parser.add_argument(
        "-o",
        "--output",
        help="Write generated Python here (default: print to stdout)",
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
    except FileNotFoundError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
