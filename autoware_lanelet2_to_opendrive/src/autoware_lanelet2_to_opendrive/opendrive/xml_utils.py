"""Utility functions for OpenDRIVE XML serialization."""

import sys

_FLOAT_MIN_NORMAL = sys.float_info.min


def replace_subnormal(value: float) -> float:
    """Return 0.0 if value is an IEEE 754 subnormal float, otherwise return it unchanged.

    Subnormals (0 < |value| < sys.float_info.min) can cause issues in some XML
    parsers and downstream tools.
    """
    if 0.0 < abs(value) < _FLOAT_MIN_NORMAL:
        return 0.0
    return value
