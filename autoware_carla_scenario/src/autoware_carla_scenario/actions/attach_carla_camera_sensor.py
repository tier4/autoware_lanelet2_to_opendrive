"""Concrete action that attaches a CARLA RGB camera sensor to an actor."""

from __future__ import annotations

from typing import Optional, Union

from ..conditions import BaseCondition
from ..entity_role import EntityRole
from ..sensor.base import CameraSensorBase
from ..sensor.carla_camera import CarlaCameraSensor, CarlaCameraSensorConfig
from .attach_camera_sensor import AttachCameraSensorAction
from .base import TickTiming


class AttachCarlaCameraSensorAction(AttachCameraSensorAction):
    """Attach a CARLA ``sensor.camera.rgb`` to an actor.

    CARLA-specific subclass of :class:`AttachCameraSensorAction`.
    Accepts a :class:`CarlaCameraSensorConfig` and creates a
    :class:`CarlaCameraSensor` on execution.

    Args:
        entity_name: ``role_name`` of the actor to attach the sensor to.
        sensor_config: CARLA camera sensor configuration.
        condition: Trigger condition (see :class:`BaseCondition`).
        timing: Tick phase (``PRE_TICK`` or ``POST_TICK``).
        label: Human-readable label for logging.
        once: If ``True`` (default) the action fires at most once.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        sensor_config: Optional[CarlaCameraSensorConfig] = None,
        condition: Optional[BaseCondition] = None,
        timing: TickTiming = TickTiming.POST_TICK,
        *,
        label: str = "attach_carla_camera_sensor",
        once: bool = True,
    ) -> None:
        resolved = sensor_config or CarlaCameraSensorConfig()
        super().__init__(
            entity_name=entity_name,
            sensor_config=resolved,
            condition=condition,
            timing=timing,
            label=label,
            once=once,
        )
        self._carla_config = resolved

    # ------------------------------------------------------------------
    # AttachCameraSensorAction interface
    # ------------------------------------------------------------------

    def _create_sensor(self) -> CameraSensorBase:
        """Create a :class:`CarlaCameraSensor`."""
        return CarlaCameraSensor(self._carla_config)
