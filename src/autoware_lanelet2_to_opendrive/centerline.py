import numpy as np
import lanelet2
from splines import CatmullRom


def extract_centerline_as_spline(
    lanelet: lanelet2.core.Lanelet, alpha: float = 0.5
) -> CatmullRom:
    """
    Extract centerline from a Lanelet and return as a CatmullRom spline object.

    Args:
        lanelet: A Lanelet2 lanelet object
        alpha: Alpha parameter for Catmull-Rom spline (0=uniform, 0.5=centripetal, 1=chordal)

    Returns:
        CatmullRom spline object that can be evaluated at any parameter t in [0, 1]
    """
    centerline = lanelet.centerline

    if len(centerline) < 2:
        raise ValueError("Lanelet must have at least 2 points in its centerline")

    points = []
    for point in centerline:
        points.append([point.x, point.y, point.z])

    points = np.array(points)

    if len(points) < 4:
        raise ValueError(
            "Lanelet must have at least 4 points for Catmull-Rom spline. Use linear interpolation for fewer points."
        )

    spline = CatmullRom(np.transpose(points), alpha=alpha)

    return spline
