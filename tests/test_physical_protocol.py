"""Checks separating planning choices from common aircraft dynamics."""
import unittest
import numpy as np
from shapely.geometry import box
from icuas_repro.experiment import measure
from icuas_repro.planner import construct, plan_team, wrap
from icuas_repro.raster import Raster
from icuas_repro.simulation import SortieEvent


class PhysicalProtocolTests(unittest.TestCase):
    def test_all_planners_share_turn_and_rendezvous_constraints(self):
        config = {'uav_speed_mps': 10., 'heading_step_deg': 5., 'physical_heading_limit': True,
                  'initial_heading_rad': 0., 'point_target_spacing_m': 100.,
                  'horizon_s': 240, 'search_iterations': 1, 'reinitializations': 1,
                  'fov_cross_m': 67., 'uavs': 1}
        raster = Raster(box(-100, -100, 800, 800), 10)
        event = SortieEvent(0, 0, 240, (0., 0.), (150., 150.))
        for method in ('point', 'naive', 'sweep', 'greedy', 'full', 'no_heading', 'no_reinit', 'neither'):
            trajectories, _ = plan_team([event], config, raster, 4, method)
            route = trajectories[0]
            np.testing.assert_allclose(route[-1, :2], event.rendezvous_point_m, atol=1e-6)
            self.assertLessEqual(np.max(np.abs(wrap(np.diff(route[:, 2])))), np.deg2rad(5)+1e-8, method)
            self.assertTrue(np.all(np.linalg.norm(np.diff(route[:, :2], axis=0), axis=1) <= 10+1e-8))

    def test_age_counts_unseen_cells_and_resets_only_observed_cells(self):
        raster = Raster(box(0, 0, 2, 2), 1, 1, 1)
        positions = np.zeros((4, 1, 3))
        positions[:, 0, :2] = (.5, .5)
        # One of four cells observed at t=0 and t=2 only.
        states = np.array([[2], [0], [2], [0]])
        diagnostics = {}
        metrics, _, _, _ = measure(raster, positions, states,
                                   {'horizon_s': 4, 'warmup_s': 2, 'observation_step_s': 1, 'uavs': 1}, diagnostics)
        np.testing.assert_allclose(diagnostics['mean_age_curve'][:, 1], [0, 1, 1.5, 2.5])
        self.assertEqual(metrics['mean_observation_age_s'], 1.25)
        self.assertEqual(metrics['post_warmup_mean_age_s'], 2.)
        self.assertEqual(metrics['terminal_p95_age_s'], 4.)
        self.assertEqual(metrics['max_censored_unobserved_time_s'], 4)
        self.assertEqual(metrics['completed_blind_interval_mean_s'], 2.)
        self.assertIsNone(metrics['time_to_80_percent_s'])


if __name__ == '__main__':
    unittest.main()
