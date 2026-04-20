"""Autoware ego vehicle — an ego vehicle not managed by TrafficManager.

This entity behaves identically to :class:`EgoVehicle` during spawn and
destroy, but signals to :class:`ScenarioRunner` that TrafficManager
autopilot must **not** be enabled on its actor.  External control (e.g.
Autoware topic I/O) will be layered on top in a future iteration.
"""

from __future__ import annotations

from .ego import EgoVehicle


class AutowareEntity(EgoVehicle):
    """Ego vehicle controlled by Autoware instead of TrafficManager.

    After spawning, the :class:`ScenarioRunner` reads
    :attr:`EgoVehicle.use_autopilot` and skips ``set_autopilot(True)``
    for this actor, leaving it free for external (Autoware) control.

    Topic I/O integration is planned for a future update.
    """

    use_autopilot: bool = False
