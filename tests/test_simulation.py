"""Event timing tests for the fixed-route temporal model."""

import unittest

from icuas_repro.route import PolylineRoute
from icuas_repro.scheduling import solve_max_flight_schedule
from icuas_repro.simulation import sortie_events


class SimulationTests(unittest.TestCase):
    def test_events_match_charge_then_flight_cycle(self):
        schedule = solve_max_flight_schedule([2], [4], pads=1, slot_seconds=1)
        route = PolylineRoute(((0.0, 0.0), (100.0, 0.0), (0.0, 0.0)))
        events = sortie_events(schedule, route, ugv_speed_mps=1.0, horizon_s=18)
        self.assertEqual(
            [(event.takeoff_s, event.landing_s) for event in events],
            [(2, 6), (8, 12), (14, 18)],
        )
        self.assertEqual(events[0].launch_point_m, (2.0, 0.0))
        self.assertEqual(events[0].rendezvous_point_m, (6.0, 0.0))

    def test_partial_final_sortie_is_excluded(self):
        schedule = solve_max_flight_schedule([2], [4], pads=1, slot_seconds=1)
        route = PolylineRoute(((0.0, 0.0), (10.0, 0.0), (0.0, 0.0)))
        events = sortie_events(schedule, route, ugv_speed_mps=1.0, horizon_s=11)
        self.assertEqual([(event.takeoff_s, event.landing_s) for event in events], [(2, 6)])


if __name__ == "__main__":
    unittest.main()
