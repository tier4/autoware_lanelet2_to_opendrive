"""Abstract base action for attaching a camera sensor to an actor.

Concrete subclasses (e.g. :class:`AttachCarlaCameraSensorAction`) implement
:meth:`_create_sensor` to return a simulator-specific
:class:`~autoware_carla_scenario.sensor.base.CameraSensorBase` instance.
"""

from __future__ import annotations

import logging
from abc import abstractmethod
from typing import TYPE_CHECKING, Optional, Union

from ..conditions import BaseCondition
from ..conditions.base import find_actor_by_role_name
from ..entity_role import EntityRole
from ..sensor.base import CameraSensorBase, CameraSensorConfig
from .base import BaseAction, TickTiming

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class AttachCameraSensorAction(BaseAction):
    """Abstract action that attaches a camera sensor to an actor.

    When the associated condition is satisfied, this action:

    1. Locates the target actor by its ``role_name``
    2. Creates a :class:`CameraSensorBase` via :meth:`_create_sensor`
    3. Calls :meth:`~CameraSensorBase.attach` to spawn the sensor

    Subclasses must implement :meth:`_create_sensor` to return the
    appropriate simulator-specific sensor instance.  This design allows
    co-simulation setups to inject cameras from different simulators
    through a common action interface.

    After execution the :attr:`sensor` property exposes the live
    :class:`CameraSensorBase` instance so that other components can read
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
        sensor_config: CameraSensorConfig,
        condition: Optional[BaseCondition] = None,
        timing: TickTiming = TickTiming.POST_TICK,
        *,
        label: str = "attach_camera_sensor",
        once: bool = True,
    ) -> None:
        super().__init__(label=label, condition=condition, timing=timing, once=once)
        self._entity_name = entity_name
        self._sensor_config = sensor_config
        self._sensor: Optional[CameraSensorBase] = None

    # ------------------------------------------------------------------
    # Public property
    # ------------------------------------------------------------------

    @property
    def sensor(self) -> Optional[CameraSensorBase]:
        """Return the attached camera sensor, or ``None`` before execution."""
        return self._sensor

    # ------------------------------------------------------------------
    # Abstract factory
    # ------------------------------------------------------------------

    @abstractmethod
    def _create_sensor(self) -> CameraSensorBase:
        """Create a simulator-specific camera sensor instance.

        Returns:
            A :class:`CameraSensorBase` subclass configured with
            :attr:`_sensor_config`.
        """
        ...

    # ------------------------------------------------------------------
    # BaseAction interface
    # ------------------------------------------------------------------

    def execute(self, world: "carla.World") -> None:
        """Locate the target actor and attach the camera sensor."""
        actor = find_actor_by_role_name(world, self._entity_name)
        if actor is None:
            logger.warning(
                "%s: actor '%s' not found",
                type(self).__name__,
                self._entity_name,
            )
            return

        camera = self._create_sensor()
        camera.attach(world, actor)
        self._sensor = camera

        logger.info(
            "%s: attached %dx%d camera to '%s'",
            type(self).__name__,
            self._sensor_config.image_width,
            self._sensor_config.image_height,
            self._entity_name,
        )
