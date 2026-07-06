"""High-level entry points that tie the frontend pipeline together.

The pipeline is::

    .osc source
        -> parser.parse_*          (py-osc2 / ANTLR parse tree)
        -> extractor.extract       (syntax IR: OscProgram)
        -> translator.translate    (semantic plan: ScenarioPlan)
        -> codegen.generate_module (readable Python source)
"""

from __future__ import annotations

from pathlib import Path

from .ast_model import OscProgram
from .codegen import generate_module
from .extractor import extract_program
from .parser import parse_osc_file, parse_osc_string
from .plan import ScenarioPlan
from .translator import translate_program


def parse_program_from_file(path: str | Path) -> OscProgram:
    """Parse and extract a ``.osc`` file into an :class:`OscProgram`."""
    return extract_program(parse_osc_file(path))


def parse_program_from_string(
    text: str, *, source_name: str = "<string>"
) -> OscProgram:
    """Parse and extract DSL source text into an :class:`OscProgram`."""
    return extract_program(parse_osc_string(text, source_name=source_name))


def plan_from_file(path: str | Path) -> ScenarioPlan:
    """Parse, extract and translate a ``.osc`` file into a :class:`ScenarioPlan`."""
    return translate_program(parse_program_from_file(path))


def plan_from_string(text: str, *, source_name: str = "<string>") -> ScenarioPlan:
    """Parse, extract and translate DSL source text into a :class:`ScenarioPlan`."""
    return translate_program(parse_program_from_string(text, source_name=source_name))


def transpile_file(path: str | Path) -> str:
    """Transpile a ``.osc`` file into readable Python scenario source.

    Args:
        path: Path to the ``.osc`` source.

    Returns:
        The generated Python module as a string.
    """
    program = parse_program_from_file(path)
    plan = translate_program(program)
    return generate_module(plan, source_name=str(path))


def transpile_string(text: str, *, source_name: str = "<string>") -> str:
    """Transpile DSL source text into readable Python scenario source."""
    program = parse_program_from_string(text, source_name=source_name)
    plan = translate_program(program)
    return generate_module(plan, source_name=source_name)


def transpile_to_file(source: str | Path, destination: str | Path) -> Path:
    """Transpile *source* (``.osc``) and write the result to *destination*.

    Args:
        source: Path to the ``.osc`` source.
        destination: Path to the ``.py`` file to write.

    Returns:
        The *destination* path.
    """
    code = transpile_file(source)
    dst = Path(destination)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(code, encoding="utf-8")
    return dst
