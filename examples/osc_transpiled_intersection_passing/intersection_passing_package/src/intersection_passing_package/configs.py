"""Config dataclass for the transpiled 'intersection_passing' scenario."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class IntersectionPassingConfig:
    """Overridable parameters (via Hydra / the scenario YAML)."""

    #: Must match the registered scenario name and the YAML ``scenario.name``.
    name: str = "intersection_passing"

    #: Fail-safe timeout in seconds.
    timeout_seconds: float = 5.0
