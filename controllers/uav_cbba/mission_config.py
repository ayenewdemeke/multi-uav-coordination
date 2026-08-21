"""Experiment constants matching the manuscript's Simulation Study."""

AGENTS = ("UAV1", "UAV2", "UAV3", "UAV4")

# name, type, x, y, priority, revisit interval, service duration
TASKS = (
    ("S1", "Safety", -8.0, -18.0, 3, 120.0, 60.0),
    ("S2", "Safety", -15.0, 10.0, 3, 150.0, 90.0),
    ("S3", "Safety", 17.0, 17.0, 3, 150.0, 90.0),
    ("S4", "Safety", -22.0, -4.0, 2, 200.0, 90.0),
    ("Q1", "Quality", -4.0, -4.0, 1, 400.0, 120.0),
    ("Q2", "Quality", 6.2, 0.0, 1, 400.0, 120.0),
    ("Q3", "Quality", 20.0, -7.0, 1, 500.0, 150.0),
    ("Q4", "Quality", 8.0, 11.0, 1, 500.0, 90.0),
)

CHARGER = (12.0, -12.0)
CRUISE_ALTITUDE = 12.0
CRUISE_SPEED = 4.0
BATTERY_CAPACITY_J = 213_444.0
POWER_W = 129.0
RESERVE_FRACTION = 0.15
CHARGER_ACCESS_S = 30.0
MISSION_DURATION_S = 7200.0

# All UAVs begin a normal deployment with fully charged batteries.
INITIAL_SOC = (1.0, 1.0, 1.0, 1.0)

LAMBDA = 0.004
BUNDLE_LIMIT = 2
AUCTION_PERIOD_S = 0.5
QUIET_ROUNDS = 2
STATUS_PERIOD_S = 0.5
ARRIVAL_RADIUS_M = 1.2
CHARGER_ARRIVAL_RADIUS_M = 3.0
TAKEOFF_ALTITUDE_M = 11.5
HOLDING_RADIUS_M = 3.0
