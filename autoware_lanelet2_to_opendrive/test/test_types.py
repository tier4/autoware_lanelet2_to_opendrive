"""Tests for type-safe Point2D and Point3D classes."""

import numpy as np
import pytest
from autoware_lanelet2_to_opendrive.types import Point2D, Point3D


class TestPoint2D:
    """Test Point2D class functionality."""

    def test_construction(self):
        """Test Point2D construction with x, y coordinates."""
        p = Point2D(1.0, 2.0)
        assert p.x == 1.0
        assert p.y == 2.0

    def test_from_array(self):
        """Test Point2D creation from numpy array."""
        arr = np.array([3.0, 4.0])
        p = Point2D.from_array(arr)
        assert p.x == 3.0
        assert p.y == 4.0

    def test_from_list(self):
        """Test Point2D creation from list."""
        p = Point2D.from_array([5.0, 6.0])
        assert p.x == 5.0
        assert p.y == 6.0

    def test_to_array(self):
        """Test conversion to numpy array."""
        p = Point2D(1.0, 2.0)
        arr = p.to_array()
        assert isinstance(arr, np.ndarray)
        assert arr.shape == (2,)
        np.testing.assert_array_equal(arr, [1.0, 2.0])

    def test_to_list(self):
        """Test conversion to list."""
        p = Point2D(1.0, 2.0)
        lst = p.to_list()
        assert lst == [1.0, 2.0]

    def test_distance_to(self):
        """Test distance calculation between two Point2D instances."""
        p1 = Point2D(0.0, 0.0)
        p2 = Point2D(3.0, 4.0)
        distance = p1.distance_to(p2)
        assert distance == pytest.approx(5.0)

    def test_immutability(self):
        """Test that Point2D is immutable (frozen dataclass)."""
        p = Point2D(1.0, 2.0)
        with pytest.raises(AttributeError):
            p.x = 5.0


class TestPoint3D:
    """Test Point3D class functionality."""

    def test_construction(self):
        """Test Point3D construction with x, y, z coordinates."""
        p = Point3D(1.0, 2.0, 3.0)
        assert p.x == 1.0
        assert p.y == 2.0
        assert p.z == 3.0

    def test_from_array(self):
        """Test Point3D creation from numpy array."""
        arr = np.array([3.0, 4.0, 5.0])
        p = Point3D.from_array(arr)
        assert p.x == 3.0
        assert p.y == 4.0
        assert p.z == 5.0

    def test_from_list(self):
        """Test Point3D creation from list."""
        p = Point3D.from_array([6.0, 7.0, 8.0])
        assert p.x == 6.0
        assert p.y == 7.0
        assert p.z == 8.0

    def test_to_array(self):
        """Test conversion to numpy array."""
        p = Point3D(1.0, 2.0, 3.0)
        arr = p.to_array()
        assert isinstance(arr, np.ndarray)
        assert arr.shape == (3,)
        np.testing.assert_array_equal(arr, [1.0, 2.0, 3.0])

    def test_to_list(self):
        """Test conversion to list."""
        p = Point3D(1.0, 2.0, 3.0)
        lst = p.to_list()
        assert lst == [1.0, 2.0, 3.0]

    def test_to_2d(self):
        """Test projection to 2D by dropping z coordinate."""
        p3d = Point3D(1.0, 2.0, 3.0)
        p2d = p3d.to_2d()
        assert isinstance(p2d, Point2D)
        assert p2d.x == 1.0
        assert p2d.y == 2.0

    def test_distance_to(self):
        """Test distance calculation between two Point3D instances."""
        p1 = Point3D(0.0, 0.0, 0.0)
        p2 = Point3D(1.0, 2.0, 2.0)
        distance = p1.distance_to(p2)
        assert distance == pytest.approx(3.0)

    def test_immutability(self):
        """Test that Point3D is immutable (frozen dataclass)."""
        p = Point3D(1.0, 2.0, 3.0)
        with pytest.raises(AttributeError):
            p.z = 10.0


class TestPointConversions:
    """Test conversions between Point2D, Point3D, and legacy formats."""

    def test_round_trip_2d_array(self):
        """Test round-trip conversion Point2D -> array -> Point2D."""
        p1 = Point2D(1.5, 2.5)
        arr = p1.to_array()
        p2 = Point2D.from_array(arr)
        assert p1 == p2

    def test_round_trip_3d_array(self):
        """Test round-trip conversion Point3D -> array -> Point3D."""
        p1 = Point3D(1.5, 2.5, 3.5)
        arr = p1.to_array()
        p2 = Point3D.from_array(arr)
        assert p1 == p2

    def test_3d_to_2d_conversion(self):
        """Test conversion from 3D to 2D."""
        p3d = Point3D(1.0, 2.0, 3.0)
        p2d = p3d.to_2d()
        assert p2d.x == p3d.x
        assert p2d.y == p3d.y
