"""Pydantic view models for the scenario result viewer."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel


class SessionSummary(BaseModel):
    """Summary of a single test session (multirun, batch, or single)."""

    date: str
    time: str
    session_type: str  # "multirun", "batch", or "single"
    scenario_name: str
    passed_count: int
    total_count: int


class SessionItem(BaseModel):
    """One scenario within a session."""

    index: int
    scenario_name: str
    passed: bool | None  # None if result is missing
    elapsed_seconds: float | None
    overrides: list[str]
    message: str = ""


class ConditionNode(BaseModel):
    """Recursive representation of a condition tree node."""

    label: str
    satisfied: bool
    message: str
    condition_type: str
    role: str
    details: dict[str, Any]
    children: list[ConditionNode] = []


class ScenarioResultView(BaseModel):
    """Full detail view of a single scenario result."""

    passed: bool | None
    message: str
    elapsed_seconds: float | None
    condition_statuses: list[ConditionNode]
    overrides: list[str]


class RunProgress(BaseModel):
    """SSE progress event payload."""

    current: int
    total: int
    scenario_name: str
    status: str  # "running", "passed", "failed", "done"
