"""Experiment constants matching the manuscript's Simulation Study."""

AGENTS = ("UAV1", "UAV2", "UAV3", "UAV4")

# name, type, x, y, priority, revisit interval, service duration
TASKS = (
    ("S1", "Safety", -8.0, -18.0, 2, 900.0, 300.0),
    ("S2", "Safety", -15.0, 10.0, 2, 900.0, 300.0),
    ("S3", "Safety", 17.0, 17.0, 2, 900.0, 300.0),
    ("S4", "Safety", -22.0, -4.0, 2, 900.0, 300.0),
    ("Q1", "Quality", -4.0, -4.0, 1, 1500.0, 300.0),
    ("Q2", "Quality", 6.2, 0.0, 1, 1500.0, 300.0),
    ("Q3", "Quality", 20.0, -7.0, 1, 1500.0, 300.0),
    ("Q4", "Quality", 8.0, 11.0, 1, 1500.0, 300.0),
)

# Each dock is 1.4 m wide. The assumed 1 m clear gap therefore gives 2.4 m
# center spacing.
CHARGERS = ((12.0, -12.0), (9.6, -12.0), (14.4, -12.0))
CHARGER = CHARGERS[0]
CRUISE_ALTITUDE = 10.0
CRUISE_SPEED = 4.0
# Conservative operational rates below the published 6 m/s normal-mode limits.
# Keep these explicit so measured site/dock rates can replace them later.
ASCENT_SPEED = 3.0
DESCENT_SPEED = 2.0
# The dock surface is 0.55 m high; the Mavic model rests with its translation
# reference approximately 0.07 m above the supporting surface.
PAD_ALTITUDE = 0.62
# DJI Mavic 3 Enterprise operational parameters.  Webots still uses the
# available Mavic 2 Pro visual model; vehicle geometry does not enter the
# task-level energy model.
BATTERY_CAPACITY_J = 77.0 * 3600.0
NOMINAL_HOVER_ENDURANCE_S = 38.0 * 60.0
POWER_W = BATTERY_CAPACITY_J / NOMINAL_HOVER_ENDURANCE_S
RESERVE_FRACTION = 0.20
TARGET_SOC = 0.90
CHARGE_20_TO_90_S = 40.0 * 60.0
CHARGE_RATE_SOC_PER_S = (TARGET_SOC - RESERVE_FRACTION) / CHARGE_20_TO_90_S
MIN_OPPORTUNISTIC_CHARGE_S = 60.0
MISSION_DURATION_S = 28800.0

# Begin at the operational charging target to avoid a one-off 100% transient.
INITIAL_SOC = (TARGET_SOC,) * len(AGENTS)
STANDBY_POINTS = ((16.8, 0.0), (0.0, 16.8),
                  (-16.8, 0.0), (0.0, -16.8))

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
# Marked-zone half-widths and half-heights, read from the Webots world.
TASK_LANDING_ZONES = (
    (-8.0, -18.0, 3.5, 1.69), (-15.0, 10.0, 4.0, 2.19),
    (17.0, 17.0, 2.1, 1.0), (-22.0, -4.0, 2.1, 1.0),
    (-4.0, -4.0, 2.5, 1.19), (6.2, 0.0, 1.25, 2.94),
    (20.0, -7.0, 1.5, 0.5), (8.0, 11.0, 1.5, 0.5),
)
# Assumed clear ground margin immediately outside a marked task zone.
TASK_LANDING_MARGIN_M = 0.5
STRUCTURE_BOUNDS = (-6.0, 6.0, -5.0, 5.0)
EMERGENCY_LANDING_MARGIN_M = 2.0
# Guards for the coordination fixes in uav_cbba.py.
AUCTION_COOLDOWN_S = 3.0         # minimum gap between auctions by one UAV
GROUND_RETRY_BACKOFF_S = 30.0    # wait after a ground charger slot is lost
AUCTION_ROUND_LIMIT = 100        # liveness cap on auction rounds
PLANNING_MARGIN_FRACTION = 0.01  # held clear of the reserve when admitting
