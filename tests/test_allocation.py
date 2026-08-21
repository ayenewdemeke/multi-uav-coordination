import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(ROOT, "controllers", "uav_cbba"))

from allocation import best_insertion, overlaps, reservation_winner, route_metrics
from mission_config import BATTERY_CAPACITY_J


class AllocationTests(unittest.TestCase):
    def test_half_open_reservations(self):
        self.assertTrue(overlaps((10, 40), (39, 50)))
        self.assertFalse(overlaps((10, 40), (40, 50)))

    def test_route_accounts_for_service_and_return(self):
        score, energy, arrival, reservation = route_metrics((0, 0), [0], 20)
        self.assertGreater(score, 0)
        self.assertGreater(energy, 129 * 60)
        self.assertEqual(reservation, (arrival, arrival + 30))

    def test_charger_conflict_rejects_insertion(self):
        candidate = best_insertion((0, 0), [], 0, 0, BATTERY_CAPACITY_J,
                                   {}, "UAV1", True)
        self.assertIsNotNone(candidate)
        conflict = {"UAV2": candidate[2]}
        self.assertIsNone(best_insertion((0, 0), [], 0, 0,
                                        BATTERY_CAPACITY_J, conflict,
                                        "UAV1", True))

    def test_lower_margin_wins(self):
        self.assertEqual(reservation_winner("UAV1", 100, "UAV2", 50), "UAV2")


if __name__ == "__main__":
    unittest.main()
