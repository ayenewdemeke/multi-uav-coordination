"""Experiment constants matching the manuscript's Simulation Study."""

AGENTS = ("UAV1", "UAV2", "UAV3", "UAV4")

# name, type, x, y, priority, revisit interval, service duration
TASKS = (
    ("S1", "Safety", -8.0, -18.0, 2, 600.0, 300.0),
    ("S2", "Safety", -15.0, 10.0, 2, 600.0, 300.0),
    ("S3", "Safety", 17.0, 17.0, 2, 600.0, 300.0),
    ("S4", "Safety", -22.0, -4.0, 2, 600.0, 300.0),
    ("Q1", "Quality", -4.0, -4.0, 1, 1200.0, 300.0),
    ("Q2", "Quality", 6.2, 0.0, 1, 1200.0, 300.0),
    ("Q3", "Quality", 20.0, -7.0, 1, 1200.0, 300.0),
    ("Q4", "Quality", 8.0, 11.0, 1, 1200.0, 300.0),
)

CHARGER = (12.0, -12.0)
CRUISE_ALTITUDE = 10.0
CRUISE_SPEED = 4.0
BATTERY_CAPACITY_J = 213_444.0
POWER_W = 120.0
RESERVE_FRACTION = 0.15
CHARGER_ACCESS_S = 30.0
MAX_SLOT_DELAY_S = 60.0
MISSION_DURATION_S = 7200.0

# All UAVs begin a normal deployment with fully charged batteries.
INITIAL_SOC = (1.0, 1.0, 1.0, 1.0)

# Exponential time discount with completion time expressed in minutes.
LAMBDA = 0.05  # min^-1
BUNDLE_LIMIT = len(TASKS)
EARLY_RELEASE_S = 15.0
QUIET_ROUNDS = 2
# Kinematic motion reaches task coordinates exactly; this only absorbs
# floating-point error in the arrival comparison.
ARRIVAL_RADIUS_M = 1e-6
CHARGER_ARRIVAL_RADIUS_M = 1e-6
TAKEOFF_ALTITUDE_M = 9.5
HOLDING_DISTANCE_M = 1.0
