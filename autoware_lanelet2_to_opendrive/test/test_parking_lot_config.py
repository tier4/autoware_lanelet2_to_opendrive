"""Unit tests for ParkingLotConfig defaults and ConversionConfig integration."""

from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    ParkingLotConfig,
)


def test_parking_lot_config_defaults():
    """ParkingLotConfig should have exactly the spec-mandated defaults."""
    cfg = ParkingLotConfig()
    assert cfg.enabled is True
    assert cfg.default_stall_width == 2.5
    assert cfg.nearest_area_threshold_m == 30.0
    assert cfg.min_area_polygon_m2 == 1.0


def test_parking_lot_config_custom_values():
    """ParkingLotConfig should accept custom values."""
    cfg = ParkingLotConfig(
        enabled=False,
        default_stall_width=3.0,
        nearest_area_threshold_m=50.0,
        min_area_polygon_m2=2.5,
    )
    assert cfg.enabled is False
    assert cfg.default_stall_width == 3.0
    assert cfg.nearest_area_threshold_m == 50.0
    assert cfg.min_area_polygon_m2 == 2.5


def test_conversion_config_has_parking_lot_field():
    """ConversionConfig should expose a parking_lot field with ParkingLotConfig defaults."""
    cfg = ConversionConfig()
    assert isinstance(cfg.parking_lot, ParkingLotConfig)
    assert cfg.parking_lot.enabled is True
    assert cfg.parking_lot.default_stall_width == 2.5
    assert cfg.parking_lot.nearest_area_threshold_m == 30.0
    assert cfg.parking_lot.min_area_polygon_m2 == 1.0


def test_conversion_config_parking_lot_override():
    """ConversionConfig should accept a custom ParkingLotConfig instance."""
    custom = ParkingLotConfig(enabled=False, default_stall_width=3.5)
    cfg = ConversionConfig(parking_lot=custom)
    assert cfg.parking_lot.enabled is False
    assert cfg.parking_lot.default_stall_width == 3.5
