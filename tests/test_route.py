"""Tests that rendezvous locations remain on the declared road polyline."""

import unittest

from icuas_repro.route import PolylineRoute


class PolylineRouteTests(unittest.TestCase):
    def setUp(self):
        self.route = PolylineRoute(((0.0, 0.0), (3.0, 0.0), (3.0, 4.0)))

    def test_length_and_segment_interpolation(self):
        self.assertEqual(self.route.length_m, 7.0)
        self.assertEqual(self.route.position_at_distance(2.0, loop=False), (2.0, 0.0))
        self.assertEqual(self.route.position_at_distance(5.0, loop=False), (3.0, 2.0))

    def test_route_wraps_for_persistent_patrol(self):
        with self.assertRaises(ValueError):
            self.route.position_at_distance(9.0)
        closed = PolylineRoute(((0., 0.), (3., 0.), (3., 4.), (0., 0.)))
        self.assertEqual(closed.position_at_distance(14.0), (2.0, 0.0))
        self.assertEqual(closed.position_at_time(7.0, 2.0), (2.0, 0.0))

    def test_non_looping_route_stops_at_last_vertex(self):
        self.assertEqual(self.route.position_at_distance(99.0, loop=False), (3.0, 4.0))


if __name__ == "__main__":
    unittest.main()
