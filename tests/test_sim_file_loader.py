import json

import pytest

from Infrastructure.Simulation import SimFileLoader


def test_loads_supported_sim_and_rasterizes_rectangles(tmp_path):
    source = tmp_path / "room.sim"
    source.write_text(json.dumps({
        "schema": "robotics_sim_lab.sim", "version": 1,
        "world": {"x_min": -2, "x_max": 4, "y_min": -3, "y_max": 5},
        "robot": {"x": 1, "y": 2, "theta": 0.5, "body_radius": .2, "safety_radius": .35},
        "map": {"obstacles": [[0, 0, 1, .5]], "grid_resolution": .5},
        "sensor": {"range": 3, "camera_fov_degrees": 120},
    }), encoding="utf-8")

    loaded = SimFileLoader().load(source)

    assert loaded.simulation_map.world_bounds == (-2, -3, 4, 5)
    assert loaded.simulation_map.robot_start_world == (1, 2)
    assert loaded.simulation_map.obstacles == ((.25, .25), (.75, .25))
    assert loaded.safety_radius == pytest.approx(.15)


def test_rejects_unknown_schema(tmp_path):
    source = tmp_path / "bad.sim"
    source.write_text('{"schema":"other","version":1}', encoding="utf-8")
    with pytest.raises(ValueError, match="Esquema"):
        SimFileLoader().load(source)
