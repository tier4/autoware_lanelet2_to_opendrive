"""Type-safe point classes for geometry operations.

This module provides immutable 2D and 3D point classes with type safety,
validation, and convenient conversion methods. These classes replace raw
list/array representations to catch dimension errors at type-check time.
"""

from dataclasses import dataclass
from typing import Union, List
import numpy as np


@dataclass(frozen=True)
class Point2D:
    """Immutable 2D point with x, y coordinates.

    This class provides type-safe representation of 2D points, replacing
    raw lists like [x, y]. Using Point2D enables:
    - Type checking to catch dimension errors
    - IDE autocomplete for x, y components
    - Clear function signatures showing expected dimensions
    - Immutability to prevent accidental modifications

    Attributes:
        x: X coordinate (easting)
        y: Y coordinate (northing)

    Example:
        ```python
        p1 = Point2D(1.0, 2.0)
        p2 = Point2D(4.0, 6.0)

        # Type-safe operations
        distance = p1.distance_to(p2)

        # Convert to numpy for calculations
        arr = p1.to_array()
        ```
    """

    x: float
    y: float

    def to_array(self) -> np.ndarray:
        """Convert to numpy array [x, y].

        Returns:
            Numpy array of shape (2,) containing x, y coordinates
        """
        return np.array([self.x, self.y])

    def to_list(self) -> List[float]:
        """Convert to list for compatibility with legacy code.

        Returns:
            List [x, y]
        """
        return [self.x, self.y]

    @classmethod
    def from_array(cls, arr: Union[np.ndarray, List[float]]) -> "Point2D":
        """Create Point2D from numpy array or list.

        Args:
            arr: Array-like object with at least 2 elements [x, y]

        Returns:
            Point2D instance with coordinates from array

        Raises:
            IndexError: If array has fewer than 2 elements

        Example:
            ```python
            p = Point2D.from_array([1.0, 2.0])
            p = Point2D.from_array(np.array([1.0, 2.0]))
            ```
        """
        return cls(float(arr[0]), float(arr[1]))

    def distance_to(self, other: "Point2D") -> float:
        """Calculate Euclidean distance to another 2D point.

        Args:
            other: Target point to measure distance to

        Returns:
            Euclidean distance between this point and other

        Example:
            ```python
            p1 = Point2D(0.0, 0.0)
            p2 = Point2D(3.0, 4.0)
            distance = p1.distance_to(p2)  # 5.0
            ```
        """
        dx = self.x - other.x
        dy = self.y - other.y
        return float(np.sqrt(dx * dx + dy * dy))


@dataclass(frozen=True)
class Point3D:
    """Immutable 3D point with x, y, z coordinates.

    This class provides type-safe representation of 3D points, replacing
    raw lists like [x, y, z]. Using Point3D enables:
    - Type checking to catch dimension errors
    - IDE autocomplete for x, y, z components
    - Clear function signatures showing expected dimensions
    - Immutability to prevent accidental modifications

    Attributes:
        x: X coordinate (easting)
        y: Y coordinate (northing)
        z: Z coordinate (elevation/altitude)

    Example:
        ```python
        p1 = Point3D(1.0, 2.0, 3.0)
        p2 = Point3D(4.0, 6.0, 8.0)

        # Type-safe operations
        distance = p1.distance_to(p2)

        # Project to 2D
        p2d = p1.to_2d()

        # Convert to numpy for calculations
        arr = p1.to_array()
        ```
    """

    x: float
    y: float
    z: float

    def to_array(self) -> np.ndarray:
        """Convert to numpy array [x, y, z].

        Returns:
            Numpy array of shape (3,) containing x, y, z coordinates
        """
        return np.array([self.x, self.y, self.z])

    def to_list(self) -> List[float]:
        """Convert to list for compatibility with legacy code.

        Returns:
            List [x, y, z]
        """
        return [self.x, self.y, self.z]

    @classmethod
    def from_array(cls, arr: Union[np.ndarray, List[float]]) -> "Point3D":
        """Create Point3D from numpy array or list.

        Args:
            arr: Array-like object with at least 3 elements [x, y, z]

        Returns:
            Point3D instance with coordinates from array

        Raises:
            IndexError: If array has fewer than 3 elements

        Example:
            ```python
            p = Point3D.from_array([1.0, 2.0, 3.0])
            p = Point3D.from_array(np.array([1.0, 2.0, 3.0]))
            ```
        """
        return cls(float(arr[0]), float(arr[1]), float(arr[2]))

    def to_2d(self) -> Point2D:
        """Project to 2D by dropping z coordinate.

        Returns:
            Point2D with x, y coordinates

        Example:
            ```python
            p3d = Point3D(1.0, 2.0, 3.0)
            p2d = p3d.to_2d()  # Point2D(1.0, 2.0)
            ```
        """
        return Point2D(self.x, self.y)

    def distance_to(self, other: "Point3D") -> float:
        """Calculate Euclidean distance to another 3D point.

        Args:
            other: Target point to measure distance to

        Returns:
            Euclidean distance between this point and other

        Example:
            ```python
            p1 = Point3D(0.0, 0.0, 0.0)
            p2 = Point3D(1.0, 2.0, 2.0)
            distance = p1.distance_to(p2)  # 3.0
            ```
        """
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return float(np.sqrt(dx * dx + dy * dy + dz * dz))


# Type alias for flexibility - functions can accept either 2D or 3D points
Point = Union[Point2D, Point3D]
