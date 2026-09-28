"""Ego vehicle driven by an external egodriver policy."""

from __future__ import annotations

import functools
import logging
from typing import TYPE_CHECKING, Callable, Optional

from ..entity.ego import EgoVehicle
from .config import EgoDriverPolicyConfig
from .policy import EgoDriverPolicy

if TYPE_CHECKING:
    import carla
    import grpc

    from ..scenario_base import BaseScenario

logger = logging.getLogger(__name__)

__all__ = ["EgoDriverEntity"]


class EgoDriverEntity(EgoVehicle):
    """Ego vehicle driven by an ``egodriver.EgodriverService`` driver.

    TrafficManager autopilot is not enabled on this actor. Instead,
    :class:`ScenarioRunner` hands every tick to an :class:`EgoDriverPolicy`,
    which queries the driver and applies the control. Any driver that serves
    the alpasim egodriver contract works, including the policies of
    ``carla_driver_interface``::

        # Terminal 1 (Python 3.11+, carla_driver_interface environment)
        carla-driver-interface serve --policy route_follower --port 50051

        # Scenario
        scenario = MyScenario(
            ego_config,
            ego_type=EgoDriverEntity.factory(
                EgoDriverPolicyConfig(driver_address="localhost:50051")
            ),
        )

    Args:
        config: Driver address, timing, sensors and route. Defaults to
            :class:`EgoDriverPolicyConfig` with its defaults.
        channel: An existing gRPC channel to the driver, used instead of
            opening one to ``config.driver_address``.
    """

    use_autopilot: bool = False

    def __init__(
        self,
        config: Optional[EgoDriverPolicyConfig] = None,
        channel: Optional["grpc.Channel"] = None,
    ) -> None:
        super().__init__()
        self.policy = EgoDriverPolicy(config, channel=channel)

    @classmethod
    def factory(
        cls,
        config: Optional[EgoDriverPolicyConfig] = None,
        channel: Optional["grpc.Channel"] = None,
    ) -> Callable[[], "EgoDriverEntity"]:
        """A zero-argument constructor to pass as ``BaseScenario(ego_type=...)``.

        :class:`ScenarioRunner` builds a fresh ego for every run, so the
        scenario holds a factory rather than an instance.
        """
        return functools.partial(cls, config, channel)

    def on_tick_loop_start(
        self, world: "carla.World", scenario: "BaseScenario"
    ) -> None:
        if self._vehicle is None:
            raise RuntimeError("the ego must be spawned before the tick loop starts")
        self.policy.start(
            world,
            self._vehicle,
            scenario_name=type(scenario).__name__,
            random_seed=scenario.random_seed,
        )

    def pre_tick(self, world: "carla.World") -> None:
        self.policy.pre_tick(world)

    def post_tick(self, world: "carla.World") -> None:
        self.policy.post_tick(world)

    def destroy(self) -> None:
        """Close the driver session and sensors, then destroy the actor."""
        try:
            self.policy.close()
        except Exception:  # noqa: BLE001 - the actor must still be destroyed
            logger.warning("failed to close the ego driver policy", exc_info=True)
        super().destroy()
