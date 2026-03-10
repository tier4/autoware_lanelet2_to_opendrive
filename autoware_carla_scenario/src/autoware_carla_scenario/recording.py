"""Scenario recording using CARLA's native recorder."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import carla


class ScenarioRecorder:
    """Records simulation using CARLA's built-in recorder.

    The native recorder captures the full simulation state (actor transforms,
    traffic lights, etc.) into a binary log that can be replayed inside CARLA
    with ``client.replay_file()``.

    Example::

        recorder = ScenarioRecorder()
        recorder.start(client, Path("output/scenario.log"))
        # ... run simulation ...
        recorder.stop(client)
    """

    def __init__(self) -> None:
        self._recording_path: Optional[Path] = None

    def start(self, client: "carla.Client", output_path: Path) -> None:
        """Start recording the simulation.

        Args:
            client: The CARLA client instance.
            output_path: Destination file path for the recording log.
        """
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._recording_path = output_path
        client.start_recorder(str(output_path))

    def stop(self, client: "carla.Client") -> None:
        """Stop the recording.

        Args:
            client: The CARLA client instance.
        """
        client.stop_recorder()
