"""Encoding CARLA camera output into what ``submit_image_observation`` carries.

CARLA hands back a raw BGRA buffer. The driver contract carries encoded bytes
(PNG or JPEG). ``carla_driver_interface`` encodes with Pillow; this package
already depends on OpenCV, which reads BGR natively, so it encodes with that
instead. The decoded pixels are the same.
"""

from __future__ import annotations

import numpy as np

from .wire import ImageFormat

__all__ = ["encode_bgra"]


def encode_bgra(
    raw: bytes, width: int, height: int, image_format: int, quality: int = 90
) -> bytes:
    """Encode CARLA's raw BGRA buffer as PNG or JPEG.

    ``image_format`` is validated by :class:`~.config.EgoDriverPolicyConfig`.
    """
    import cv2  # noqa: PLC0415 - heavy import, needed only when cameras are on

    expected = width * height * 4
    if len(raw) != expected:
        raise ValueError(
            f"expected {expected} bytes for a {width}x{height} BGRA image, "
            f"got {len(raw)}"
        )
    bgr = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
    if image_format == ImageFormat.JPEG:
        ok, encoded = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        )
    else:
        # Compression level 1: the encoder is in the closed loop, so speed
        # matters far more than a few percent of frame size.
        ok, encoded = cv2.imencode(".png", bgr, [int(cv2.IMWRITE_PNG_COMPRESSION), 1])
    if not ok:
        raise RuntimeError("OpenCV failed to encode a camera frame")
    return bytes(encoded.tobytes())
