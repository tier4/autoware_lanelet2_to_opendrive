"""Unit tests for EntityRole."""

from __future__ import annotations

import pytest

from autoware_carla_scenario import EntityRole


class TestEntityRoleConstruction:
    """Tests for EntityRole construction and validation."""

    def test_ego_factory(self) -> None:
        role = EntityRole.ego()
        assert str(role) == "Ego"

    def test_npc_factory(self) -> None:
        role = EntityRole.npc(1)
        assert str(role) == "npc1"

    def test_npc_factory_large_number(self) -> None:
        role = EntityRole.npc(42)
        assert str(role) == "npc42"

    def test_npc_factory_rejects_zero(self) -> None:
        with pytest.raises(ValueError, match="positive integer"):
            EntityRole.npc(0)

    def test_npc_factory_rejects_negative(self) -> None:
        with pytest.raises(ValueError, match="positive integer"):
            EntityRole.npc(-1)

    def test_direct_construction_ego(self) -> None:
        role = EntityRole("Ego")
        assert str(role) == "Ego"

    def test_direct_construction_npc(self) -> None:
        role = EntityRole("npc1")
        assert str(role) == "npc1"

    def test_direct_construction_custom_category(self) -> None:
        role = EntityRole("vehicle2")
        assert str(role) == "vehicle2"

    def test_direct_construction_pedestrian(self) -> None:
        role = EntityRole("pedestrian3")
        assert str(role) == "pedestrian3"

    def test_rejects_lowercase_ego(self) -> None:
        with pytest.raises(ValueError, match="Invalid entity role name"):
            EntityRole("ego")

    def test_rejects_uppercase_category(self) -> None:
        with pytest.raises(ValueError, match="Invalid entity role name"):
            EntityRole("NPC1")

    def test_rejects_underscore(self) -> None:
        with pytest.raises(ValueError, match="Invalid entity role name"):
            EntityRole("npc_1")

    def test_rejects_leading_zero(self) -> None:
        with pytest.raises(ValueError, match="Invalid entity role name"):
            EntityRole("npc01")

    def test_rejects_zero_suffix(self) -> None:
        with pytest.raises(ValueError, match="Invalid entity role name"):
            EntityRole("npc0")

    def test_rejects_empty_string(self) -> None:
        with pytest.raises(ValueError, match="Invalid entity role name"):
            EntityRole("")

    def test_rejects_number_only(self) -> None:
        with pytest.raises(ValueError, match="Invalid entity role name"):
            EntityRole("123")

    def test_rejects_category_only(self) -> None:
        with pytest.raises(ValueError, match="Invalid entity role name"):
            EntityRole("npc")

    def test_rejects_spaces(self) -> None:
        with pytest.raises(ValueError, match="Invalid entity role name"):
            EntityRole("npc 1")


class TestEntityRoleEquality:
    """Tests for equality and hashing."""

    def test_equal_roles(self) -> None:
        assert EntityRole("npc1") == EntityRole("npc1")

    def test_unequal_roles(self) -> None:
        assert EntityRole("npc1") != EntityRole("npc2")

    def test_ego_equality(self) -> None:
        assert EntityRole.ego() == EntityRole("Ego")

    def test_npc_factory_equality(self) -> None:
        assert EntityRole.npc(1) == EntityRole("npc1")

    def test_hash_consistency(self) -> None:
        a = EntityRole("npc1")
        b = EntityRole("npc1")
        assert hash(a) == hash(b)

    def test_usable_as_dict_key(self) -> None:
        d = {EntityRole.npc(1): "first", EntityRole.ego(): "ego"}
        assert d[EntityRole("npc1")] == "first"
        assert d[EntityRole("Ego")] == "ego"

    def test_usable_in_set(self) -> None:
        s = {EntityRole.npc(1), EntityRole.npc(1), EntityRole.ego()}
        assert len(s) == 2

    def test_not_equal_to_string(self) -> None:
        assert EntityRole("npc1") != "npc1"


class TestEntityRoleStringConversion:
    """Tests for __str__ and __repr__."""

    def test_str(self) -> None:
        assert str(EntityRole.npc(1)) == "npc1"
        assert str(EntityRole.ego()) == "Ego"

    def test_repr(self) -> None:
        assert repr(EntityRole.npc(1)) == "EntityRole('npc1')"
        assert repr(EntityRole.ego()) == "EntityRole('Ego')"
