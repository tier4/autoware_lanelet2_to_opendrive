"""MP4 recording of scenario camera frames."""

from __future__ import annotations

from pathlib import Path
from typing import List, Tuple

import numpy as np


class ScenarioRecorder:
    """Collects RGB frames and saves them as an MP4 video file.

    Uses ``cv2.VideoWriter`` from the *opencv-python* package.
    """

    def __init__(
        self,
        fps: float = 20.0,
        resolution: Tuple[int, int] = (1280, 720),
    ) -> None:
        """Initialize the recorder.

        Args:
            fps: Frames per second for the output video.
            resolution: Output video resolution as (width, height).
        """
        self.fps = fps
        self.resolution = resolution
        self._frames: List[np.ndarray] = []

    def add_frame(self, image: np.ndarray) -> None:
        """Append a single RGB frame to the internal buffer.

        Args:
            image: RGB image as a NumPy array (H×W×3, uint8).
        """
        self._frames.append(image)

    def save(self, output_path: Path) -> None:
        """Write all buffered frames to an MP4 file.

        The frames are converted from RGB to BGR before encoding because
        OpenCV's :class:`cv2.VideoWriter` expects BGR byte order.

        Args:
            output_path: Destination file path (will be created/overwritten).
        """
        import cv2

        output_path.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(
            str(output_path),
            fourcc,
            self.fps,
            self.resolution,
        )
        try:
            for frame in self._frames:
                # Resize to target resolution if needed
                h, w = frame.shape[:2]
                tw, th = self.resolution
                if (w, h) != (tw, th):
                    frame = cv2.resize(frame, (tw, th))
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                writer.write(bgr)
        finally:
            writer.release()

    def clear(self) -> None:
        """Discard all buffered frames."""
        self._frames.clear()
