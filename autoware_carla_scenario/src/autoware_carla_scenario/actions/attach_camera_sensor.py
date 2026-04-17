"""Action that attaches a CARLA camera sensor to an actor when a condition is met."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional, Union

from ..conditions import BaseCondition
from ..conditions.base import find_actor_by_role_name
from ..entity_role import EntityRole
from ..sensor.carla_camera import CarlaCameraSensor, CarlaCameraSensorConfig
from .base import BaseAction, TickTiming

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class AttachCarlaCameraSensorAction(BaseAction):
    """Attach a CARLA RGB camera sensor to an actor.

    When the associated condition is satisfied, this action:

    1. Locates the target actor by its ``role_name``
    2. Spawns and attaches a :class:`CarlaCameraSensor` configured by the
       provided :class:`CarlaCameraSensorConfig`

    After execution the :attr:`sensor` property exposes the live
    :class:`CarlaCameraSensor` instance so that other components can read
    frames or intrinsics.

    Args:
        entity_name: ``role_name`` of the actor to attach the sensor to.
        sensor_config: Camera sensor configuration.
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
        label: str = "attach_camera_sensor",
        once: bool = True,
    ) -> None:
        super().__init__(label=label, condition=condition, timing=timing, once=once)
        self._entity_name = entity_name
        self._sensor_config = sensor_config or CarlaCameraSensorConfig()
        self._sensor: Optional[CarlaCameraSensor] = None

    # ------------------------------------------------------------------
    # Public property
    # ------------------------------------------------------------------

    @property
    def sensor(self) -> Optional[CarlaCameraSensor]:
        """Return the attached camera sensor, or ``None`` before execution."""
        return self._sensor

    # ------------------------------------------------------------------
    # BaseAction interface
    # ------------------------------------------------------------------

    def execute(self, world: "carla.World") -> None:
        """Attach the camera sensor to the target actor."""
        actor = find_actor_by_role_name(world, self._entity_name)
        if actor is None:
            logger.warning(
                "AttachCarlaCameraSensorAction: actor '%s' not found",
                self._entity_name,
            )
            return

        camera = CarlaCameraSensor(self._sensor_config)
        camera.attach(world, actor)
        self._sensor = camera

        logger.info(
            "AttachCarlaCameraSensorAction: attached %dx%d camera to '%s'",
            self._sensor_config.image_width,
            self._sensor_config.image_height,
            self._entity_name,
        )
