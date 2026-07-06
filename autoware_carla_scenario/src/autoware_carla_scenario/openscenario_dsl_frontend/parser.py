"""Thin wrapper around ``py-osc2`` that parses OpenSCENARIO DSL sources.

This module isolates the ``osc2parser`` / ANTLR dependency behind a small,
typed surface.  It returns the raw ANTLR parse tree together with the parser's
``ruleNames`` table (needed to interpret rule indices) so that
:mod:`.extractor` can walk the tree without importing ANTLR itself.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .errors import OscDependencyError, OscParseError, SyntaxErrorInfo


@dataclass(frozen=True)
class ParseResult:
    """The outcome of parsing an OpenSCENARIO DSL source.

    Attributes:
        tree: The ANTLR ``osc_file`` parse-tree root.
        rule_names: The parser's ``ruleNames`` list, used to resolve a rule
            node's index to its grammar rule name.
        source_name: A human-readable identifier for the source (file path or
            ``"<string>"``).
    """

    tree: Any
    rule_names: list[str]
    source_name: str


def _load_antlr() -> tuple[Any, Any, Any, Any, Any]:
    """Import the ANTLR runtime and generated ``py-osc2`` classes.

    Returns:
        A tuple ``(InputStream, CommonTokenStream, FileStream, Lexer, Parser)``.

    Raises:
        OscDependencyError: If ``py-osc2`` (or its ANTLR runtime) is missing.
    """
    try:
        from antlr4 import CommonTokenStream, FileStream, InputStream
        from osc2parser.openscenario2Lexer import openscenario2Lexer
        from osc2parser.openscenario2Parser import openscenario2Parser
    except ImportError as exc:  # pragma: no cover - exercised via error path
        raise OscDependencyError(
            "The 'py-osc2' package is required to parse OpenSCENARIO DSL "
            "sources but is not installed. It cannot be a normal dependency of "
            "this project because it pins antlr4-python3-runtime==4.7.1, which "
            "conflicts with the 4.9.* required by hydra/omegaconf. Install it "
            "in an isolated environment (e.g. 'pip install py-osc2' in a "
            "dedicated venv) to run the transpiler. See "
            "docs/openscenario_dsl_frontend.md for details."
        ) from exc
    return (
        InputStream,
        CommonTokenStream,
        FileStream,
        openscenario2Lexer,
        openscenario2Parser,
    )


def _make_error_listener() -> tuple[Any, list[SyntaxErrorInfo]]:
    """Create an ANTLR error listener that collects syntax errors.

    Returns:
        A tuple ``(listener, errors)`` where *errors* is the list the listener
        appends to as it observes syntax errors.
    """
    from antlr4.error.ErrorListener import ErrorListener

    errors: list[SyntaxErrorInfo] = []

    class _CollectingListener(ErrorListener):  # type: ignore[misc]
        def syntaxError(  # noqa: N802 - ANTLR interface name
            self,
            recognizer: Any,
            offending_symbol: Any,
            line: int,
            column: int,
            msg: str,
            e: Any,
        ) -> None:
            errors.append(SyntaxErrorInfo(line=line, column=column, message=msg))

    return _CollectingListener(), errors


def _parse(stream: Any, source_name: str) -> ParseResult:
    """Run the lexer/parser over *stream* and raise on syntax errors."""
    _, common_token_stream, _, lexer_cls, parser_cls = _load_antlr()

    lexer = lexer_cls(stream)
    tokens = common_token_stream(lexer)
    parser = parser_cls(tokens)
    parser.removeErrorListeners()
    listener, errors = _make_error_listener()
    parser.addErrorListener(listener)

    tree = parser.osc_file()
    if errors:
        raise OscParseError(source_name, errors)
    return ParseResult(
        tree=tree, rule_names=list(parser.ruleNames), source_name=source_name
    )


def parse_osc_file(path: str | Path) -> ParseResult:
    """Parse an OpenSCENARIO DSL (``.osc``) file.

    Args:
        path: Path to the ``.osc`` source file.

    Returns:
        The :class:`ParseResult`.

    Raises:
        OscDependencyError: If ``py-osc2`` is not installed.
        OscParseError: If the source contains syntax errors.
        FileNotFoundError: If *path* does not exist.
    """
    file_path = Path(path)
    if not file_path.is_file():
        raise FileNotFoundError(f"OpenSCENARIO DSL file not found: {file_path}")
    input_stream_cls, _, file_stream_cls, _, _ = _load_antlr()
    del input_stream_cls
    stream = file_stream_cls(str(file_path), "utf-8")
    return _parse(stream, source_name=str(file_path))


def parse_osc_string(text: str, *, source_name: str = "<string>") -> ParseResult:
    """Parse OpenSCENARIO DSL from an in-memory string.

    Args:
        text: The DSL source. A trailing newline is added if missing, which the
            indentation-aware lexer requires to close the final block.
        source_name: A label used in error messages.

    Returns:
        The :class:`ParseResult`.

    Raises:
        OscDependencyError: If ``py-osc2`` is not installed.
        OscParseError: If the source contains syntax errors.
    """
    input_stream_cls, _, _, _, _ = _load_antlr()
    if not text.endswith("\n"):
        text = text + "\n"
    stream = input_stream_cls(text)
    return _parse(stream, source_name=source_name)
