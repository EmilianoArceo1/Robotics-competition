import json

import numpy as np

from Infrastructure.Simulation import SimFileLoader
from Logic.Competition.environment import CompetitionConfig, CompetitionWorld, load_competition_map


def test_map_conversion_matches_official_block_reduction(tmp_path):
    source = tmp_path / "occ_map.npy"
    np.save(source, np.array([[254, 254, 0], [254, 254, 254], [0, 254, 254]], dtype=np.uint8))
    result = load_competition_map(source, padding=1)
    assert result.shape == (4, 4)
    assert np.array_equal(result[1:-1, 1:-1], [[0, 1], [1, 1]])


def test_world_uses_official_start_delays_and_labels(tmp_path):
    source = tmp_path / "occ_map.npy"
    np.save(source, np.full((80, 80), 254, dtype=np.uint8))
    config = CompetitionConfig(pd_size=5, start_pose=(10, 10), num_robots=3,
                               num_laser=16, lidar_range=1, max_steps=2)
    world = CompetitionWorld(source, config)
    assert [robot.start_delay for robot in world.robots] == [0, 5, 10]
    assert world.robots[0].pose.tolist() == [15, 15]
    world.step()
    assert set(np.unique(world.robots[0].combined_obs_map)).issubset({0.0, 0.5, 1.0})
    assert world.timestep == 1


def test_all_packaged_competition_manifests_are_loadable():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1] / "Assets" / "competition_maps"
    loaded = [SimFileLoader().load(root / f"env{i}" / f"env{i}.sim") for i in range(1, 8)]
    assert all(item.competition_occ_map is not None for item in loaded)
    assert [item.competition_config["max_steps"] for item in loaded] == [500, 500, 1000, 1000, 1000, 1500, 1500]
