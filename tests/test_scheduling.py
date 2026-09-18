"""Tests for the paper's periodic charging constraint."""

import unittest

from icuas_repro.scheduling import solve_max_flight_schedule


class SchedulingTests(unittest.TestCase):
    def test_heterogeneous_objective_counts_common_horizon(self):
        # A 3/3 robot and a 1/2 robot cannot share one pad periodically.
        # Over six slots they work 3 versus 4 slots, despite 3 > 2 per cycle.
        schedule = solve_max_flight_schedule([180, 60], [180, 120], pads=1)
        self.assertEqual(schedule.selected_uavs, (1,))

    def test_tie_break_prefers_lower_indices_then_phases(self):
        schedule = solve_max_flight_schedule([60]*3, [60]*3, pads=1)
        self.assertEqual(schedule.selected_uavs, (0, 1))
        self.assertEqual(dict(schedule.charge_start_slot), {0: 0, 1: 1})

    def test_three_identical_uavs_fit_on_two_pads(self):
        schedule = solve_max_flight_schedule([840] * 3, [1200] * 3, pads=2)
        self.assertEqual(schedule.horizon_slots, 34)
        self.assertEqual(schedule.selected_uavs, (0, 1, 2))
        for time_slot in range(schedule.horizon_slots):
            self.assertLessEqual(sum(schedule.is_charging(uav, time_slot)
                                     for uav in schedule.selected_uavs), 2)

    def test_one_pad_can_stagger_two_half_cycle_chargers(self):
        schedule = solve_max_flight_schedule([60, 60], [60, 60], pads=1)
        self.assertEqual(schedule.selected_uavs, (0, 1))
        for time_slot in range(schedule.horizon_slots):
            self.assertLessEqual(sum(schedule.is_charging(uav, time_slot)
                                     for uav in schedule.selected_uavs), 1)


if __name__ == "__main__":
    unittest.main()
