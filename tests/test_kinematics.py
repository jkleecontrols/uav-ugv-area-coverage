"""Independent endpoint/speed/curvature and rasterization checks."""
import unittest
import numpy as np
from shapely.geometry import box, Point

from icuas_repro.coverage import footprint_polygon, Observation, FOV
from icuas_repro.planner import return_segments, sample_segments, wrap
from icuas_repro.raster import Raster
from icuas_repro.scenario import load_scenario


class KinematicsTests(unittest.TestCase):
    def test_arc_tangent_returns_and_exact_duration(self):
        rng = np.random.default_rng(910)
        for _ in range(100):
            pose = rng.uniform(-1000, 1000, size=3)
            pose[2] = rng.uniform(-np.pi, np.pi)
            end = rng.uniform(-1000, 1000, size=2)
            radius, speed = 120., 10.
            back = return_segments(pose, end, radius)
            duration = int(np.ceil((sum(l for l, _ in back)+2*np.pi*radius)/speed))+1
            surplus = duration*speed-sum(l for l, _ in back)
            trajectory = sample_segments(pose, [(surplus, 2*np.pi/surplus)]+back, speed, duration)
            np.testing.assert_allclose(trajectory[-1, :2], end, atol=1e-6)
            steps = np.linalg.norm(np.diff(trajectory[:, :2], axis=0), axis=1)
            self.assertTrue(np.all(steps <= speed+1e-8))
            self.assertTrue(np.all(steps > 9.99))
            self.assertLessEqual(np.max(np.abs(wrap(np.diff(trajectory[:, 2])))), speed/radius+1e-8)

    def test_window_raster_matches_polygon_reference(self):
        aoi = box(0, 0, 50, 40)
        raster = Raster(aoi, 1, forward=12, cross=8)
        for pose in ((10, 10, 0), (0, 2, .43), (40, 20, 1.27), (99, 99, 0)):
            polygon = footprint_polygon(Observation(*pose), FOV(12, 8))
            expected = [i for i in range(raster.size)
                        if polygon.covers(Point(raster.x[i % len(raster.x)], raster.y[i // len(raster.x)]))]
            self.assertEqual(set(raster.footprint(pose)), set(expected))

    def test_campus_route_is_closed_and_on_source_roads(self):
        from shapely.ops import unary_union
        from shapely.geometry import LineString
        config, aoi, roads, route, _ = load_scenario()
        self.assertEqual(route.vertices[0], route.vertices[-1])
        road_area = unary_union(roads).buffer(1e-6)
        for a, b in zip(route.vertices, route.vertices[1:]):
            self.assertTrue(road_area.covers(LineString([a, b])))


if __name__ == '__main__':
    unittest.main()
