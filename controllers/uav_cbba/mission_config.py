"""Experiment constants matching the manuscript's Simulation Study."""

import os as _os

# The world always instantiates the full roster; the active fleet is the first
# CR_CBBA_UAV_COUNT of them.  Controllers outside the active set stay parked.
ROSTER = ("UAV1", "UAV2", "UAV3", "UAV4", "UAV5", "UAV6")
UAV_COUNT = int(_os.environ.get("CR_CBBA_UAV_COUNT", "4"))
if not 1 <= UAV_COUNT <= len(ROSTER):
    raise ValueError("UAV count must be between 1 and %d" % len(ROSTER))
AGENTS = ROSTER[:UAV_COUNT]

# name, type, x, y, priority, revisit interval, service duration
TASKS = (
    ("S1", "Safety", -8.0, -18.0, 2, 900.0, 300.0),
    ("S2", "Safety", -15.0, 10.0, 2, 900.0, 300.0),
    ("S3", "Safety", 17.0, 17.0, 2, 900.0, 300.0),
    ("S4", "Safety", -22.0, -4.0, 2, 900.0, 300.0),
    ("S5", "Safety", -18.0, 18.0, 2, 900.0, 300.0),
    ("S6", "Safety", 10.0, -20.0, 2, 900.0, 300.0),
    ("Q1", "Quality", -4.0, -4.0, 1, 1500.0, 300.0),
    ("Q2", "Quality", 6.2, 0.0, 1, 1500.0, 300.0),
    ("Q3", "Quality", 20.0, -7.0, 1, 1500.0, 300.0),
    ("Q4", "Quality", 8.0, 11.0, 1, 1500.0, 300.0),
    ("Q5", "Quality", -10.0, 6.0, 1, 1500.0, 300.0),
    ("Q6", "Quality", 18.0, 4.0, 1, 1500.0, 300.0),
)

# Each dock is 1.4 m wide. The assumed 1 m clear gap therefore gives 2.4 m
# center spacing.  Pads are indexed left to right along the apron.
CHARGERS = ((7.2, -12.0), (9.6, -12.0),
            (12.0, -12.0), (14.4, -12.0))
CHARGER = CHARGERS[0]
DEFAULT_CHARGER_COUNT = 3
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
MISSION_DURATION_S = 28800.0

# Begin at the operational charging target to avoid a one-off 100% transient.
INITIAL_SOC = (TARGET_SOC,) * len(AGENTS)
# UAVs park in a single line on the apron, 4.8 m in front of the charger row.
# 1.5 m spacing clears the ~0.9 m rotor span.  The line extends to the right as
# UAVs are added.
STANDBY_SPACING_M = 1.5
STANDBY_ORIGIN = (0.0, -16.8)
STANDBY_POINTS = tuple(
    (STANDBY_ORIGIN[0] + index * STANDBY_SPACING_M, STANDBY_ORIGIN[1])
    for index in range(len(AGENTS)))

# Exponential time discount with completion time expressed in minutes.
LAMBDA = 0.05  # min^-1
BUNDLE_LIMIT = len(TASKS)
# Advance-release allowance (delta in III.A).  One service duration of
# lookahead, so a UAV finishing a bundle can see the next due task
# instead of idling until it is almost late.
EARLY_RELEASE_S = 300.0
QUIET_ROUNDS = 2
# Kinematic motion reaches task coordinates exactly; this only absorbs
# floating-point error in the arrival comparison.
ARRIVAL_RADIUS_M = 1e-6
CHARGER_ARRIVAL_RADIUS_M = 1e-6
TAKEOFF_ALTITUDE_M = 9.5
HOLDING_DISTANCE_M = 1.0
AUCTION_ROUND_LIMIT = 100        # liveness cap on auction rounds
