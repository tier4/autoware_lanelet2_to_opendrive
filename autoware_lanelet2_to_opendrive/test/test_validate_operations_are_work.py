"""A validate-only preprocessing config must still run.

``main.py`` decides whether to run the preprocessing stage from a hand-written
list of the operation fields, and that list is missing
``validate_operations``. A map config whose only operation is a validation is
therefore seen as having no preprocessing at all, and the check it asked for
never runs -- silently, because validations are informational and produce no
output of their own when skipped.

The two lists also disagree with each other: ``PreprocessOperation`` derives
``has_operations`` from ``OPERATION_TYPES``, which does include
``validate_operations``, so the origin check demands an origin for exactly the
config the runner then skips.

The lanelet ids below are deliberately absent from the fixture: a validation
that runs says so either way, and "lanelet not found" needs no map content.
"""

import subprocess
from pathlib import Path

import pytest

from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    OPERATION_TYPES,
    PreprocessOperation,
    ValidateOperation,
)

FIXTURE = (Path(__file__).parent / "data" / "walkway_mini.osm").resolve()


def test_every_operation_type_counts_as_work() -> None:
    """``has_operations`` must be derived, so a new operation cannot be missed."""
    for name in OPERATION_TYPES:
        config = PreprocessOperation(input_map_path="in.osm", output_map_path="out.osm")
        setattr(config, name, [object()])
        assert config.has_operations, f"{name} is not counted as preprocessing work"


@pytest.mark.xfail(strict=True, reason="validate_operations is not counted as work")
def test_a_validate_only_config_runs_the_validation(tmp_path: Path) -> None:
    """The conversion must report the validation it was asked to perform."""
    operation = ValidateOperation(first_lanelet_id=1, second_lanelet_id=2)
    out = tmp_path / "validate_only.xodr"
    result = subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=example",
            "target=carla",
            f"input_map_path={FIXTURE}",
            f"output_map_path={out}",
            "map.validate_operations=[{"
            f"first_lanelet_id:{operation.first_lanelet_id},"
            f"second_lanelet_id:{operation.second_lanelet_id}"
            "}]",
        ],
        capture_output=True,
        text=True,
    )
    # Hydra logs to stdout, so both streams matter.
    output = result.stdout + result.stderr
    assert (
        result.returncode == 0
    ), f"conversion exited {result.returncode}\n{output[-2000:]}"
    assert "Executing validate operation" in output, (
        "the validate operation never ran, so the preprocessing stage was"
        f" skipped for a config that asked for work:\n{output[-2000:]}"
    )
