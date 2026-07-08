"""Relative-position action: hold a moving longitudinal gap to another actor.

Unlike the coarse TrafficManager manoeuvres (:class:`TurnAction`,
:class:`LaneChangeAction`) which issue a command once, this action runs **every
tick** (``once=False``) and closes a *dynamic* relative-position goal such as
"stay 20 m ahead of the ego": both vehicles move, so the target is recomputed
each frame.

Only the **longitudinal** channel is controlled — the controlled vehicle stays
on TrafficManager autopilot for lane keeping, and each tick its desired cruise
speed is nudged by a proportional law on the gap error via
``TrafficManager.set_desired_speed``. This keeps the controller cheap and avoids
fighting TrafficManager's lateral control.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional, Union

from ..conditions import BaseCondition
from ..conditions.base import find_actor_in_list
from ..constants import DEFAULT_TM_PORT
from ..entity_role import EntityRole
from ..kinematics import Vector3
from .base import BaseAction, TickTiming

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)

#: Magnitude below which a reference forward vector is considered degenerate.
_NEAR_ZERO_THRESHOLD = 1e-6


class RelativePositionAction(BaseAction):
    """Drive an actor toward a longitudinal gap relative to a reference actor.

    Each tick the action measures the signed longitudinal gap of the controlled
    entity relative to the reference entity's forward direction and adjusts the
    controlled entity's desired cruise speed with a proportional law:

    ``gap   = (entity_pos - reference_pos) · reference_forward``
    ``error = target_gap - gap``
    ``desired_speed = clamp(reference_speed + gain * error, 0, max_speed)``

    A positive ``target_gap`` means *ahead of* the reference; negative means
    *behind*. Lateral positioning (which lane) is left to TrafficManager.

    Args:
        entity_name: ``role_name`` of the controlled vehicle.
        reference_name: ``role_name`` of the reference vehicle the gap is
            measured against.
        target_gap: Desired signed longitudinal gap in metres (``+`` ahead of,
            ``-`` behind the reference).
        client: A ``carla.Client`` used to obtain the TrafficManager.
        condition: Activation condition; while satisfied the controller runs
            each tick. Defaults to always-on.
        timing: Tick phase (``PRE_TICK`` by default).
        label: Human-readable identifier.
        once: Kept ``False`` — a relative-position goal is a continuous
            controller, not a one-shot command.
        gain: Proportional gain in km/h of speed correction per metre of error.
        max_speed_kmh: Upper clamp on the commanded desired speed.
        tm_port: TrafficManager port.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        reference_name: Union[EntityRole, str],
        target_gap: float,
        client: "carla.Client",
        condition: Optional[BaseCondition] = None,
        timing: TickTiming = TickTiming.PRE_TICK,
        *,
        label: str = "relative_position",
        once: bool = False,
        gain: float = 2.0,
        max_speed_kmh: float = 60.0,
        tm_port: int = DEFAULT_TM_PORT,
    ) -> None:
        super().__init__(label=label, condition=condition, timing=timing, once=once)
        self._entity_name = entity_name
        self._reference_name = reference_name
        self._target_gap = target_gap
        self._client = client
        self._gain = gain
        self._max_speed_kmh = max_speed_kmh
        self._tm_port = tm_port

    # ------------------------------------------------------------------
    # BaseAction interface
    # ------------------------------------------------------------------

    def execute(self, world: "carla.World") -> None:
        """Nudge the controlled entity's desired speed toward the target gap."""
        actors = world.get_actors()
        entity = find_actor_in_list(actors, self._entity_name)
        reference = find_actor_in_list(actors, self._reference_name)
        if entity is None or reference is None:
            logger.warning(
                "RelativePositionAction: entity '%s' or reference '%s' not found",
                self._entity_name,
                self._reference_name,
            )
            return

        desired_kmh = self._desired_speed_kmh(entity, reference)
        if desired_kmh is None:
            return

        tm = self._client.get_trafficmanager(self._tm_port)
        tm.set_desired_speed(entity, desired_kmh)
        logger.debug(
            "RelativePositionAction: '%s' desired speed -> %.1f km/h "
            "(target gap %.1f m to '%s')",
            self._entity_name,
            desired_kmh,
            self._target_gap,
            self._reference_name,
        )

    # ------------------------------------------------------------------
    # Control law
    # ------------------------------------------------------------------

    def _desired_speed_kmh(
        self, entity: "carla.Actor", reference: "carla.Actor"
    ) -> Optional[float]:
        """Compute the commanded desired speed (km/h), or ``None`` if undefined."""
        ref_tf = reference.get_transform()
        fwd_carla = ref_tf.get_forward_vector()
        fwd = Vector3(fwd_carla.x, fwd_carla.y, 0.0)
        fwd_mag = fwd.magnitude()
        if fwd_mag < _NEAR_ZERO_THRESHOLD:
            return None
        fwd_unit = fwd / fwd_mag

        ent_loc = entity.get_location()
        ref_loc = reference.get_location()
        displacement = Vector3(ent_loc.x - ref_loc.x, ent_loc.y - ref_loc.y, 0.0)
        gap = displacement.dot(fwd_unit)
        error = self._target_gap - gap

        ref_speed_kmh = Vector3.from_carla_vector3d(
            reference.get_velocity()
        ).magnitude()
        ref_speed_kmh *= 3.6

        desired = ref_speed_kmh + self._gain * error
        return max(0.0, min(desired, self._max_speed_kmh))
