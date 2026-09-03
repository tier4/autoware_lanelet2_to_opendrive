"""Unit tests for the OpenDRIVE-usage decorator + source scan.

CARLA-free: exercises the ``requires_opendrive`` decorator and the AST scan with
locally-declared symbols, so it needs no live framework import.
"""

from __future__ import annotations

import pytest

from autoware_carla_scenario.opendrive_lint import (
    check_scenario_source,
    find_opendrive_symbols,
    opendrive_symbols,
    requires_opendrive,
)


@requires_opendrive
class _MyOdCondition:
    pass


@requires_opendrive
def _my_od_helper() -> None:
    pass


def test_decorator_registers_and_marks() -> None:
    assert "_MyOdCondition" in opendrive_symbols()
    assert "_my_od_helper" in opendrive_symbols()
    assert getattr(_MyOdCondition, "requires_opendrive", False) is True
    assert getattr(_my_od_helper, "requires_opendrive", False) is True


def test_find_flags_referenced_symbol() -> None:
    source = "class S:\n    def setup(self):\n        _MyOdCondition()\n"
    assert find_opendrive_symbols(source) == {"_MyOdCondition"}


def test_find_resolves_import_aliases() -> None:
    source = "from somewhere import _MyOdCondition as Foo\n\nFoo()\n"
    assert find_opendrive_symbols(source) == {"_MyOdCondition"}


def test_find_ignores_unregistered_names() -> None:
    # EntityInAreaCondition is deliberately NOT registered (it is OpenDRIVE-free).
    source = "class S:\n    def setup(self):\n        EntityInAreaCondition()\n"
    assert find_opendrive_symbols(source) == set()


def test_check_raises_for_opendrive_usage() -> None:
    with pytest.raises(ValueError, match="OpenDRIVE"):
        check_scenario_source("_my_od_helper()", "S")


def test_check_passes_without_opendrive_usage() -> None:
    check_scenario_source("x = 1\n", "S")  # must not raise
