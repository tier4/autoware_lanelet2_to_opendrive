"""Unit tests for the kinematics package (Vector3, velocity, acceleration types)."""

from __future__ import annotations


import pytest

from autoware_carla_scenario.kinematics import (
    AbsoluteAcceleration,
    AbsoluteVelocity,
    CoordinateFrame,
    FrameMismatchError,
    FrenetAcceleration,
    FrenetVelocity,
    RelativeAcceleration,
    RelativeVelocity,
    Vector3,
)

CARLA = CoordinateFrame.CARLA_WORLD
LL2 = CoordinateFrame.LANELET2


# ============================================================================
# Vector3
# ============================================================================


class TestVector3:
    def test_add(self) -> None:
        assert Vector3(1, 2, 3) + Vector3(4, 5, 6) == Vector3(5, 7, 9)

    def test_sub(self) -> None:
        assert Vector3(4, 5, 6) - Vector3(1, 2, 3) == Vector3(3, 3, 3)

    def test_mul(self) -> None:
        assert Vector3(1, 2, 3) * 2 == Vector3(2, 4, 6)

    def test_rmul(self) -> None:
        assert 3 * Vector3(1, 2, 3) == Vector3(3, 6, 9)

    def test_truediv(self) -> None:
        assert Vector3(2, 4, 6) / 2 == Vector3(1, 2, 3)

    def test_truediv_by_zero(self) -> None:
        with pytest.raises(ZeroDivisionError):
            Vector3(1, 2, 3) / 0

    def test_neg(self) -> None:
        assert -Vector3(1, -2, 3) == Vector3(-1, 2, -3)

    def test_magnitude(self) -> None:
        assert Vector3(3, 4, 0).magnitude() == pytest.approx(5.0)

    def test_normalized(self) -> None:
        n = Vector3(0, 0, 5).normalized()
        assert n == Vector3(0, 0, 1)

    def test_normalized_zero_raises(self) -> None:
        with pytest.raises(ValueError, match="zero"):
            Vector3.zero().normalized()

    def test_dot(self) -> None:
        assert Vector3(1, 2, 3).dot(Vector3(4, 5, 6)) == pytest.approx(32.0)

    def test_cross(self) -> None:
        assert Vector3(1, 0, 0).cross(Vector3(0, 1, 0)) == Vector3(0, 0, 1)

    def test_zero(self) -> None:
        assert Vector3.zero() == Vector3(0.0, 0.0, 0.0)

    def test_frozen(self) -> None:
        v = Vector3(1, 2, 3)
        with pytest.raises(AttributeError):
            v.x = 10  # type: ignore[misc]

    def test_not_implemented_for_wrong_type(self) -> None:
        assert Vector3(1, 2, 3).__add__("bad") is NotImplemented
        assert Vector3(1, 2, 3).__mul__("bad") is NotImplemented
        assert Vector3(1, 2, 3).__truediv__("bad") is NotImplemented


# ============================================================================
# AbsoluteVelocity
# ============================================================================


class TestAbsoluteVelocity:
    def test_from_components(self) -> None:
        v = AbsoluteVelocity.from_components(10, 0, 0, CARLA)
        assert v.vector == Vector3(10, 0, 0)
        assert v.frame == CARLA

    def test_speed(self) -> None:
        v = AbsoluteVelocity.from_components(3, 4, 0, CARLA)
        assert v.speed() == pytest.approx(5.0)

    def test_add_relative(self) -> None:
        abs_v = AbsoluteVelocity.from_components(10, 0, 0, CARLA)
        rel_v = RelativeVelocity.from_components(5, 0, 0, CARLA)
        result = abs_v + rel_v
        assert isinstance(result, AbsoluteVelocity)
        assert result.vector == Vector3(15, 0, 0)

    def test_sub_absolute_gives_relative(self) -> None:
        v1 = AbsoluteVelocity.from_components(10, 0, 0, CARLA)
        v2 = AbsoluteVelocity.from_components(3, 0, 0, CARLA)
        result = v1 - v2
        assert isinstance(result, RelativeVelocity)
        assert result.vector == Vector3(7, 0, 0)

    def test_sub_relative_gives_absolute(self) -> None:
        abs_v = AbsoluteVelocity.from_components(10, 0, 0, CARLA)
        rel_v = RelativeVelocity.from_components(3, 0, 0, CARLA)
        result = abs_v - rel_v
        assert isinstance(result, AbsoluteVelocity)
        assert result.vector == Vector3(7, 0, 0)

    def test_scalar_mul(self) -> None:
        v = AbsoluteVelocity.from_components(1, 2, 3, CARLA)
        assert (v * 2).vector == Vector3(2, 4, 6)
        assert (2 * v).vector == Vector3(2, 4, 6)

    def test_scalar_div(self) -> None:
        v = AbsoluteVelocity.from_components(4, 6, 8, CARLA)
        assert (v / 2).vector == Vector3(2, 3, 4)

    def test_neg(self) -> None:
        v = AbsoluteVelocity.from_components(1, -2, 3, CARLA)
        assert (-v).vector == Vector3(-1, 2, -3)

    def test_zero(self) -> None:
        v = AbsoluteVelocity.zero(CARLA)
        assert v.speed() == pytest.approx(0.0)
        assert v.frame == CARLA

    def test_frame_mismatch_add(self) -> None:
        abs_v = AbsoluteVelocity.from_components(1, 0, 0, CARLA)
        rel_v = RelativeVelocity.from_components(1, 0, 0, LL2)
        with pytest.raises(FrameMismatchError):
            abs_v + rel_v

    def test_frame_mismatch_sub(self) -> None:
        v1 = AbsoluteVelocity.from_components(1, 0, 0, CARLA)
        v2 = AbsoluteVelocity.from_components(1, 0, 0, LL2)
        with pytest.raises(FrameMismatchError):
            v1 - v2

    def test_frozen(self) -> None:
        v = AbsoluteVelocity.from_components(1, 2, 3, CARLA)
        with pytest.raises(AttributeError):
            v.frame = LL2  # type: ignore[misc]


# ============================================================================
# RelativeVelocity
# ============================================================================


class TestRelativeVelocity:
    def test_between(self) -> None:
        target = AbsoluteVelocity.from_components(10, 5, 0, CARLA)
        ref = AbsoluteVelocity.from_components(3, 2, 0, CARLA)
        rel = RelativeVelocity.between(target, ref)
        assert rel.vector == Vector3(7, 3, 0)
        assert rel.frame == CARLA

    def test_between_frame_mismatch(self) -> None:
        v1 = AbsoluteVelocity.from_components(1, 0, 0, CARLA)
        v2 = AbsoluteVelocity.from_components(1, 0, 0, LL2)
        with pytest.raises(FrameMismatchError):
            RelativeVelocity.between(v1, v2)

    def test_add_relative(self) -> None:
        r1 = RelativeVelocity.from_components(1, 0, 0, CARLA)
        r2 = RelativeVelocity.from_components(2, 0, 0, CARLA)
        result = r1 + r2
        assert isinstance(result, RelativeVelocity)
        assert result.vector == Vector3(3, 0, 0)

    def test_add_absolute(self) -> None:
        rel = RelativeVelocity.from_components(5, 0, 0, CARLA)
        abs_v = AbsoluteVelocity.from_components(10, 0, 0, CARLA)
        result = rel + abs_v
        assert isinstance(result, AbsoluteVelocity)
        assert result.vector == Vector3(15, 0, 0)

    def test_sub(self) -> None:
        r1 = RelativeVelocity.from_components(5, 3, 0, CARLA)
        r2 = RelativeVelocity.from_components(2, 1, 0, CARLA)
        result = r1 - r2
        assert isinstance(result, RelativeVelocity)
        assert result.vector == Vector3(3, 2, 0)

    def test_neg_swaps_direction(self) -> None:
        rel = RelativeVelocity.from_components(5, -3, 0, CARLA)
        neg = -rel
        assert neg.vector == Vector3(-5, 3, 0)

    def test_speed(self) -> None:
        rel = RelativeVelocity.from_components(3, 4, 0, CARLA)
        assert rel.speed() == pytest.approx(5.0)


# ============================================================================
# FrenetVelocity
# ============================================================================


class TestFrenetVelocity:
    def test_arithmetic(self) -> None:
        f1 = FrenetVelocity(10.0, 1.0)
        f2 = FrenetVelocity(5.0, -0.5)
        assert f1 + f2 == FrenetVelocity(15.0, 0.5)
        assert f1 - f2 == FrenetVelocity(5.0, 1.5)
        assert f1 * 2 == FrenetVelocity(20.0, 2.0)
        assert 2 * f1 == FrenetVelocity(20.0, 2.0)
        assert f1 / 2 == FrenetVelocity(5.0, 0.5)
        assert -f1 == FrenetVelocity(-10.0, -1.0)

    def test_speed(self) -> None:
        fv = FrenetVelocity(3.0, 4.0)
        assert fv.speed() == pytest.approx(5.0)

    def test_zero(self) -> None:
        fv = FrenetVelocity.zero()
        assert fv.speed() == pytest.approx(0.0)

    def test_div_by_zero(self) -> None:
        with pytest.raises(ZeroDivisionError):
            FrenetVelocity(1.0, 2.0) / 0


# ============================================================================
# AbsoluteAcceleration
# ============================================================================


class TestAbsoluteAcceleration:
    def test_from_components(self) -> None:
        a = AbsoluteAcceleration.from_components(1, 2, 3, CARLA)
        assert a.vector == Vector3(1, 2, 3)
        assert a.frame == CARLA

    def test_add_relative(self) -> None:
        abs_a = AbsoluteAcceleration.from_components(1, 0, 0, CARLA)
        rel_a = RelativeAcceleration.from_components(2, 0, 0, CARLA)
        result = abs_a + rel_a
        assert isinstance(result, AbsoluteAcceleration)
        assert result.vector == Vector3(3, 0, 0)

    def test_sub_absolute_gives_relative(self) -> None:
        a1 = AbsoluteAcceleration.from_components(5, 0, 0, CARLA)
        a2 = AbsoluteAcceleration.from_components(2, 0, 0, CARLA)
        result = a1 - a2
        assert isinstance(result, RelativeAcceleration)
        assert result.vector == Vector3(3, 0, 0)

    def test_sub_relative_gives_absolute(self) -> None:
        abs_a = AbsoluteAcceleration.from_components(5, 0, 0, CARLA)
        rel_a = RelativeAcceleration.from_components(2, 0, 0, CARLA)
        result = abs_a - rel_a
        assert isinstance(result, AbsoluteAcceleration)
        assert result.vector == Vector3(3, 0, 0)

    def test_frame_mismatch(self) -> None:
        a1 = AbsoluteAcceleration.from_components(1, 0, 0, CARLA)
        a2 = AbsoluteAcceleration.from_components(1, 0, 0, LL2)
        with pytest.raises(FrameMismatchError):
            a1 - a2

    def test_scalar_ops(self) -> None:
        a = AbsoluteAcceleration.from_components(1, 2, 3, CARLA)
        assert (a * 3).vector == Vector3(3, 6, 9)
        assert (3 * a).vector == Vector3(3, 6, 9)
        assert (a / 2).vector == Vector3(0.5, 1.0, 1.5)

    def test_magnitude(self) -> None:
        a = AbsoluteAcceleration.from_components(3, 4, 0, CARLA)
        assert a.magnitude() == pytest.approx(5.0)


# ============================================================================
# RelativeAcceleration
# ============================================================================


class TestRelativeAcceleration:
    def test_between(self) -> None:
        t = AbsoluteAcceleration.from_components(5, 3, 0, CARLA)
        r = AbsoluteAcceleration.from_components(2, 1, 0, CARLA)
        rel = RelativeAcceleration.between(t, r)
        assert rel.vector == Vector3(3, 2, 0)

    def test_add_relative(self) -> None:
        r1 = RelativeAcceleration.from_components(1, 0, 0, CARLA)
        r2 = RelativeAcceleration.from_components(2, 0, 0, CARLA)
        result = r1 + r2
        assert isinstance(result, RelativeAcceleration)

    def test_add_absolute(self) -> None:
        rel = RelativeAcceleration.from_components(1, 0, 0, CARLA)
        abs_a = AbsoluteAcceleration.from_components(5, 0, 0, CARLA)
        result = rel + abs_a
        assert isinstance(result, AbsoluteAcceleration)
        assert result.vector == Vector3(6, 0, 0)

    def test_neg(self) -> None:
        rel = RelativeAcceleration.from_components(1, -2, 3, CARLA)
        assert (-rel).vector == Vector3(-1, 2, -3)


# ============================================================================
# FrenetAcceleration
# ============================================================================


class TestFrenetAcceleration:
    def test_arithmetic(self) -> None:
        a1 = FrenetAcceleration(2.0, 1.0)
        a2 = FrenetAcceleration(1.0, -0.5)
        assert a1 + a2 == FrenetAcceleration(3.0, 0.5)
        assert a1 - a2 == FrenetAcceleration(1.0, 1.5)
        assert a1 * 3 == FrenetAcceleration(6.0, 3.0)
        assert a1 / 2 == FrenetAcceleration(1.0, 0.5)
        assert -a1 == FrenetAcceleration(-2.0, -1.0)

    def test_magnitude(self) -> None:
        a = FrenetAcceleration(3.0, 4.0)
        assert a.magnitude() == pytest.approx(5.0)


# ============================================================================
# Affine-space round-trip
# ============================================================================


class TestAffineSpaceRoundTrip:
    """Verify that the affine-space identities hold."""

    def test_abs_minus_abs_plus_abs_equals_original(self) -> None:
        v1 = AbsoluteVelocity.from_components(10, 5, 2, CARLA)
        v2 = AbsoluteVelocity.from_components(3, 1, 0, CARLA)
        rel = v1 - v2  # RelativeVelocity
        reconstructed = v2 + rel  # AbsoluteVelocity
        assert reconstructed.vector.x == pytest.approx(v1.vector.x)
        assert reconstructed.vector.y == pytest.approx(v1.vector.y)
        assert reconstructed.vector.z == pytest.approx(v1.vector.z)

    def test_negate_relative_reverses_direction(self) -> None:
        v1 = AbsoluteVelocity.from_components(10, 0, 0, CARLA)
        v2 = AbsoluteVelocity.from_components(3, 0, 0, CARLA)
        rel_forward = RelativeVelocity.between(v1, v2)  # v1 - v2
        rel_backward = -rel_forward  # v2 - v1
        assert (v1 + rel_backward).vector.x == pytest.approx(v2.vector.x)
