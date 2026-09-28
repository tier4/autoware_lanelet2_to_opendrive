"""Route publishing, matching what alpasim's ``RouteGenerator`` does.

alpasim projects the ego onto the map, takes the stretch of route ahead, and
sends it to the driver **in the rig frame at the current timestamp**
(``events/policy.py``).  :class:`RouteProvider` does the same for a route
polyline expressed in the ``local`` frame, and holds no CARLA references so it
can be tested directly. Ported from ``carla_driver_interface.runtime.route``.
"""

from __future__ import annotations

import numpy as np

from . import polyline
from .geometry import Pose

__all__ = ["RouteProvider"]


class RouteProvider:
    """Slices a global route into the window the driver sees each step.

    Progress along the route is monotonic and advances **only** in
    :meth:`waypoints_in_rig`. Everything else here is a pure read: a second
    mutator would let the marker move more than once per step, and because
    :meth:`_project` searches a bounded window it does not converge in one call,
    so a repeat call on the same pose can jump the marker forward again.
    """

    def __init__(
        self,
        route_in_local: np.ndarray,
        horizon_m: float = 80.0,
        resolution_m: float = 2.0,
    ) -> None:
        route = np.asarray(route_in_local, dtype=np.float64).reshape(-1, 3)
        if len(route) < 2:
            raise ValueError("a route needs at least two points")
        self._route = route
        self._arc = polyline.arc_lengths(route)
        self.horizon_m = horizon_m
        self.resolution_m = resolution_m
        self._progress_m = 0.0

    @property
    def total_length_m(self) -> float:
        return float(self._arc[-1])

    @property
    def completion(self) -> float:
        """Fraction of the route driven so far, in ``[0, 1]``."""
        if self.total_length_m <= 0.0:
            return 1.0
        return min(1.0, self._progress_m / self.total_length_m)

    def lateral_error_m(self, pose_local_to_rig: Pose) -> float:
        """Signed offset of the ego from the route centreline. Does not advance.

        Positive means the ego is to the left of the route. This is the honest
        tracking metric: the driver's plan is anchored on the ego, so measuring
        the ego against *that* is structurally zero.
        """
        nearest = self._sample(self._project(pose_local_to_rig.position))
        return float(pose_local_to_rig.inverse().transform_points(nearest)[0][1])

    def waypoints_in_rig(self, pose_local_to_rig: Pose) -> np.ndarray:
        """The route ahead of ``pose_local_to_rig``, resampled, in the rig frame.

        Advances the progress marker; call once per policy step.
        """
        self._progress_m = self._project(pose_local_to_rig.position)

        end = min(self.total_length_m, self._progress_m + self.horizon_m)
        distances: np.ndarray = np.arange(self._progress_m, end, self.resolution_m)
        # Always finish on the exact endpoint. Without it the last stride is
        # truncated, the ego can never reach 100% completion, and the rollout
        # runs off the end of the route into undefined behaviour.
        distances = np.append(distances, end)
        if len(distances) < 2:
            distances = np.array([max(0.0, end - self.resolution_m), end])

        points_local = self._sample(distances)
        return pose_local_to_rig.inverse().transform_points(points_local)

    # -- internals ---------------------------------------------------------

    def _project(self, position_local: np.ndarray) -> float:
        """Arc length of the closest route point, searching forward only.

        The window is generous enough to absorb a step's worth of motion plus a
        large tracking error, but bounded so a loop route never matches
        backwards onto an earlier lap.
        """
        search_end = min(
            self.total_length_m, self._progress_m + max(self.horizon_m, 30.0)
        )
        samples = np.append(
            np.arange(self._progress_m, search_end, self.resolution_m), search_end
        )
        points = self._sample(samples)
        squared = np.sum((points[:, :2] - position_local[:2]) ** 2, axis=1)
        return float(samples[int(np.argmin(squared))])

    def _sample(self, distances: np.ndarray | float) -> np.ndarray:
        return polyline.sample(self._route, self._arc, distances)
