"""Regression tests for the shared coverage definitions.

Run with: ``python -m unittest discover -s tests -v``.
"""

import math
import unittest

from shapely.geometry import Polygon

from icuas_repro.coverage import FOV, Observation, evaluate_coverage, footprint_polygon


class CoverageTests(unittest.TestCase):
    def setUp(self):
        self.aoi = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
        self.fov = FOV(forward_m=4.0, cross_track_m=4.0)

    def test_footprint_rotates_about_observation(self):
        footprint = footprint_polygon(Observation(5.0, 5.0, math.pi / 2), FOV(6.0, 2.0))
        min_x, min_y, max_x, max_y = footprint.bounds
        self.assertAlmostEqual(min_x, 4.0)
        self.assertAlmostEqual(max_x, 6.0)
        self.assertAlmostEqual(min_y, 2.0)
        self.assertAlmostEqual(max_y, 8.0)

    def test_one_observation_has_no_redundancy(self):
        metrics = evaluate_coverage(
            self.aoi, [Observation(5.0, 5.0, 0.0)], self.fov, grid_resolution_m=1.0
        )
        self.assertEqual(metrics.aoi_cells, 100)
        self.assertEqual(metrics.covered_cells, 16)
        self.assertEqual(metrics.cell_observations, 16)
        self.assertEqual(metrics.coverage_percent, 16.0)
        self.assertEqual(metrics.redundancy_percent, 0.0)

    def test_identical_observations_are_fifty_percent_redundant(self):
        observation = Observation(5.0, 5.0, 0.0)
        metrics = evaluate_coverage(self.aoi, [observation, observation], self.fov)
        self.assertEqual(metrics.covered_cells, 16)
        self.assertEqual(metrics.cell_observations, 32)
        self.assertEqual(metrics.redundancy_percent, 50.0)

    def test_empty_observation_list_is_valid(self):
        metrics = evaluate_coverage(self.aoi, [], self.fov)
        self.assertEqual(metrics.covered_cells, 0)
        self.assertEqual(metrics.coverage_percent, 0.0)
        self.assertEqual(metrics.redundancy_percent, 0.0)


if __name__ == "__main__":
    unittest.main()
