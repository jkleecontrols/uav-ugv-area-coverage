"""Feasible planning, ablation isolation and temporal coverage integration."""
import unittest
import numpy as np
from shapely.geometry import box

from icuas_repro.experiment import measure
from icuas_repro.planner import plan_team, wrap
from icuas_repro.raster import Raster
from icuas_repro.simulation import SortieEvent


class PipelineTests(unittest.TestCase):
    def test_mission_ending_before_first_takeoff(self):
        raster = Raster(box(0, 0, 10, 10), 1)
        trajectories, log = plan_team([], {}, raster, 0, 'full')
        self.assertEqual(trajectories, [])
        self.assertEqual(log[0]['covered_planning_cells'], 0)

    def test_seed_repeatability_and_reinitialization_isolation(self):
        raster = Raster(box(-100, -100, 800, 800), 10)
        config = {'uav_speed_mps': 10., 'heading_step_deg': 5.,
                  'initial_heading_rad': 0., 'point_target_spacing_m': 150.,
                  'horizon_s': 180, 'search_iterations': 1, 'reinitializations': 0}
        events = [SortieEvent(0, 0, 180, (0., 0.), (100., 100.))]
        a, log_a = plan_team(events, config, raster, 7, 'full')
        b, log_b = plan_team(events, config, raster, 7, 'full')
        c, log_c = plan_team(events, config, raster, 7, 'no_reinit')
        np.testing.assert_array_equal(a[0], b[0])
        np.testing.assert_array_equal(a[0], c[0])
        self.assertEqual(log_a, log_b)
        self.assertEqual(log_a, log_c)
        np.testing.assert_allclose(a[0][-1, :2], (100, 100), atol=1e-6)
        self.assertLessEqual(np.max(np.abs(wrap(np.diff(a[0][:, 2])))), np.deg2rad(5)+1e-9)

    def test_simultaneous_cameras_do_not_create_revisit_episode(self):
        raster = Raster(box(0, 0, 4, 4), 1, 2, 2)
        positions = np.zeros((3, 2, 3))
        positions[:, :, :2] = (2, 2)
        states = np.full((3, 2), 2)
        config = {'horizon_s': 2, 'observation_step_s': 1, 'uavs': 2}
        metric, counts, episodes, _ = measure(raster, positions, states, config)
        self.assertEqual(metric['covered_cells'], 4)
        self.assertEqual(metric['cell_observations'], 16)
        self.assertEqual(metric['exposure_redundancy_percent'], 75)
        self.assertEqual(metric['coverage_episodes'], 4)
        self.assertEqual(metric['revisited_covered_cells_percent'], 0)

    def test_absent_second_starts_a_new_episode(self):
        raster = Raster(box(0, 0, 4, 4), 1, 2, 2)
        positions = np.zeros((3, 1, 3))
        positions[:, :, :2] = (2, 2)
        positions[1, :, :2] = (100, 100)
        states = np.full((3, 1), 2)
        metric, _, _, _ = measure(raster, positions, states,
                                  {'horizon_s': 3, 'observation_step_s': 1, 'uavs': 1})
        self.assertEqual(metric['coverage_episodes'], 8)
        self.assertEqual(metric['revisited_covered_cells_percent'], 100)


if __name__ == '__main__':
    unittest.main()
