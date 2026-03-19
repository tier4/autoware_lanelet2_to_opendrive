"""RGB camera recorder that attaches to a CARLA actor and writes MP4 video.

Frames are piped directly to *ffmpeg* as raw BGR data and encoded to
H.264 (``libx264``) in real time, producing a browser-compatible MP4
file without any post-processing step.
"""

from __future__ import annotations

import logging
import subprocess
import threading
from pathlib import Path

import carla
import numpy as np

logger = logging.getLogger(__name__)

#: Default image width in pixels.
DEFAULT_IMAGE_WIDTH: int = 1920
#: Default image height in pixels.
DEFAULT_IMAGE_HEIGHT: int = 1080
#: Default camera horizontal field of view in degrees.
DEFAULT_FOV: float = 90.0
#: Default recording frame rate (matches CARLA synchronous mode at 20 Hz).
DEFAULT_FPS: float = 20.0


class CameraRecorder:
    """Attach an RGB camera sensor to a CARLA actor and record video to MP4.

    The camera is placed at the given offset relative to the actor's local
    frame, producing the same viewpoint as the spectator camera when
    configured with matching *offset_back*, *offset_up*, and *pitch*.

    Internally, raw BGR frames are streamed to an *ffmpeg* subprocess via
    stdin and encoded to H.264 (``yuv420p``) so the resulting ``.mp4``
    file is playable in all modern web browsers.

    Args:
        world: The CARLA world instance.
        actor: The actor to attach the camera to (typically the ego vehicle).
        output_path: Destination path for the output ``.mp4`` file.
        offset_back: Distance behind the actor in metres.
        offset_up: Height above the actor in metres.
        pitch: Camera pitch angle in degrees (negative = look down).
        image_width: Image width in pixels.
        image_height: Image height in pixels.
        fov: Horizontal field of view in degrees.
        fps: Recording frame rate.
    """

    def __init__(
        self,
        world: "carla.World",
        actor: "carla.Actor",
        output_path: Path,
        *,
        offset_back: float = 8.0,
        offset_up: float = 5.0,
        pitch: float = -15.0,
        image_width: int = DEFAULT_IMAGE_WIDTH,
        image_height: int = DEFAULT_IMAGE_HEIGHT,
        fov: float = DEFAULT_FOV,
        fps: float = DEFAULT_FPS,
    ) -> None:
        self._lock = threading.Lock()
        self._output_path = output_path
        self._frame_count = 0

        # Set up the RGB camera blueprint
        bp_lib = world.get_blueprint_library()
        camera_bp = bp_lib.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(image_width))
        camera_bp.set_attribute("image_size_y", str(image_height))
        camera_bp.set_attribute("fov", str(fov))

        # Place the camera at the same relative position as the spectator.
        # In the actor's local frame: -x = behind, +z = above.
        transform = carla.Transform(
            carla.Location(x=-offset_back, y=0.0, z=offset_up),
            carla.Rotation(pitch=pitch),
        )
        self._sensor: "carla.Actor | None" = world.spawn_actor(
            camera_bp, transform, attach_to=actor
        )

        # Launch ffmpeg to encode raw BGR frames to H.264 MP4 in real time.
        self._ffmpeg: subprocess.Popen[bytes] | None = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-f",
                "rawvideo",
                "-vcodec",
                "rawvideo",
                "-s",
                f"{image_width}x{image_height}",
                "-pix_fmt",
                "bgr24",
                "-r",
                str(fps),
                "-i",
                "-",
                "-c:v",
                "libx264",
                "-preset",
                "fast",
                "-crf",
                "23",
                "-pix_fmt",
                "yuv420p",
                str(output_path),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )

        # Start listening for frames
        self._sensor.listen(self._on_image)
        logger.info(
            "CameraRecorder started: %dx%d @ %.0f fps -> %s",
            image_width,
            image_height,
            fps,
            output_path,
        )

    def _on_image(self, image: "carla.Image") -> None:
        """Callback invoked by CARLA for each captured frame."""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))  # BGRA
        # Drop the alpha channel to get BGR and ensure contiguous memory.
        bgr = np.ascontiguousarray(array[:, :, :3])

        with self._lock:
            if self._ffmpeg is not None and self._ffmpeg.stdin is not None:
                try:
                    self._ffmpeg.stdin.write(bgr.tobytes())
                    self._frame_count += 1
                except BrokenPipeError:
                    pass

    def stop(self) -> None:
        """Stop the sensor, finalise the ffmpeg process, and destroy the sensor actor."""
        if self._sensor is not None:
            self._sensor.stop()
            self._sensor.destroy()
            self._sensor = None

        with self._lock:
            if self._ffmpeg is not None and self._ffmpeg.stdin is not None:
                self._ffmpeg.stdin.close()

        if self._ffmpeg is not None:
            self._ffmpeg.wait()
            if self._ffmpeg.returncode != 0:
                stderr_io = self._ffmpeg.stderr
                stderr = stderr_io.read() if stderr_io is not None else b""
                logger.warning(
                    "ffmpeg exited with code %d: %s",
                    self._ffmpeg.returncode,
                    stderr.decode(errors="replace")[-500:],
                )
            self._ffmpeg = None

        logger.info(
            "CameraRecorder stopped: %d frames written to %s",
            self._frame_count,
            self._output_path,
        )
