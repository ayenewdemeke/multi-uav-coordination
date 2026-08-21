import importlib.util
import os
import unittest

ROOT = os.path.dirname(os.path.dirname(__file__))
spec = importlib.util.spec_from_file_location(
    "analysis", os.path.join(ROOT, "analysis", "analyze_results.py"))
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


class AnalysisTests(unittest.TestCase):
    def test_summary_uses_logged_events(self):
        rows = [
            {"event": "task_complete", "task": "S1", "time": 100, "agent": "UAV1"},
            {"event": "charger_conflict", "time": 200, "agent": "UAV2"},
            {"event": "charger_wait_end", "duration_s": 12.5, "time": 213, "agent": "UAV2"},
            {"event": "charger_conflict_resolved", "time": 300, "agent": "UAV3"},
            {"event": "allocation_cycle", "convergence_s": .2, "time": 0, "agent": "UAV1"},
            {"event": "mission_end", "battery_soc": .2, "messages": 10,
             "time": 3600, "agent": "UAV1"},
        ]
        result = analysis.summarize("baseline", rows)
        self.assertEqual(result["charger_conflicts"], 1)
        self.assertEqual(result["charger_waiting_s"], 12.5)
        self.assertEqual(result["charging_reassignments"], 1)
        self.assertEqual(result["completed_services"], 1)
        self.assertEqual(result["communication_messages"], 10)


if __name__ == "__main__": unittest.main()
