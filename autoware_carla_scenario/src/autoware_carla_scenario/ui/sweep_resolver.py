"""Resolve sweep constraints into concrete override batches.

Replicates the constraint matching + binding resolution from
:class:`LaneletConstraintSweeper` without launching any jobs.
This is lightweight (no CARLA import) and runs in the viewer process.
"""

from __future__ import annotations

import logging

from hydra import compose, initialize_config_dir
from hydra.core.global_hydra import GlobalHydra
from omegaconf import OmegaConf

from autoware_carla_scenario.examples.run import _CONF_DIR

logger = logging.getLogger(__name__)


def resolve_sweep(
    scenario_name: str,
    extra_overrides: list[str] | None = None,
) -> list[list[str]]:
    """Resolve sweep constraints and return a list of override lists.

    Each inner list contains the Hydra overrides for a single job,
    e.g. ``["scenario=X", "ego.spawn_lanelet_id=5", "ego.spawn_s=18.6"]``.

    Args:
        scenario_name: A concrete (non-glob) scenario config name.
        extra_overrides: Additional CLI overrides (e.g. ``["map=nishishinjuku"]``).

    Returns:
        A list of override lists, one per matched lanelet.
        Empty list if no lanelets match or the config has no sweep section.
    """
    from autoware_carla_scenario.sweeper.bindings import (  # noqa: PLC0415
        parse_binding,
    )
    from autoware_carla_scenario.sweeper.constraints import (  # noqa: PLC0415
        create_routing_graph,
        find_matching_lanelets,
        parse_constraint,
    )
    from autoware_carla_scenario.sweeper.map_loader import (  # noqa: PLC0415
        load_lanelet2_map,
    )

    # -- 1. Compose the Hydra config ------------------------------------
    GlobalHydra.instance().clear()
    with initialize_config_dir(config_dir=str(_CONF_DIR), version_base=None):
        cfg = compose(
            config_name="config",
            overrides=[f"scenario={scenario_name}", *(extra_overrides or [])],
        )

    # -- 2. Check for sweep section -------------------------------------
    sweep_cfg = OmegaConf.select(cfg, "sweep")
    if sweep_cfg is None:
        logger.info("No 'sweep' section in config for %s.", scenario_name)
        return []

    sweep_dict = OmegaConf.to_container(sweep_cfg, resolve=True)
    assert isinstance(sweep_dict, dict)  # noqa: S101

    constraints_cfg = sweep_dict.get("constraints", {})
    if not constraints_cfg:
        logger.warning("sweep.constraints is empty for %s.", scenario_name)
        return []

    # -- 3. Resolve map paths -------------------------------------------
    lanelet2_path = OmegaConf.select(cfg, "map.lanelet2_path")
    xodr_path = OmegaConf.select(cfg, "map.xodr_path")
    if lanelet2_path is None or xodr_path is None:
        raise ValueError(
            f"Sweep resolution requires map.lanelet2_path and map.xodr_path "
            f"for scenario {scenario_name}."
        )

    # -- 4. Load lanelet2 map (lightweight, no CARLA) -------------------
    lanelet_map = load_lanelet2_map(lanelet2_path, xodr_path)

    # -- 5. Parse constraints and find matching lanelets ----------------
    all_constraints = []
    constraint_target_key: str | None = None
    for target_key, constraint_list in constraints_cfg.items():
        constraint_target_key = target_key
        for c_cfg in constraint_list:
            all_constraints.append(parse_constraint(c_cfg))

    if not all_constraints or constraint_target_key is None:
        return []

    routing_graph = create_routing_graph(lanelet_map)
    matched_ids = find_matching_lanelets(all_constraints, lanelet_map, routing_graph)
    if not matched_ids:
        logger.warning("No lanelets match constraints for %s.", scenario_name)
        return []

    # -- 6. Resolve bindings for each matched lanelet -------------------
    bindings_cfg = sweep_dict.get("bindings", {})
    bindings = [parse_binding(k, v) for k, v in bindings_cfg.items()]

    batches: list[list[str]] = []
    for lid in matched_ids:
        overrides = [
            f"scenario={scenario_name}",
            f"{constraint_target_key}={lid}",
        ]
        for binding in bindings:
            try:
                value = binding.resolve(lid, lanelet_map, routing_graph)
                overrides.append(f"{binding.target_key}={value}")
            except Exception:
                logger.warning(
                    "Binding %s failed for lanelet %d; skipping.",
                    binding.target_key,
                    lid,
                    exc_info=True,
                )
                break
        else:
            batches.append(overrides)

    logger.info(
        "Resolved %d job(s) for %s: %s",
        len(batches),
        scenario_name,
        [b[1] for b in batches],  # show lanelet ids
    )
    return batches
