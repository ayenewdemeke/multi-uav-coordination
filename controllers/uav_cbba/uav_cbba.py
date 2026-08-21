"""Persistent charging-constrained CBBA experiment controller.

Select the matched experiment with CR_CBBA_MODE=baseline or proposed.
"""
import json
import math
import os
import time
from pathlib import Path

from controller import Robot
from allocation import best_insertion, distance, overlaps, route_metrics
from mission_config import *


def clamp(v, lo, hi): return max(lo, min(v, hi))


def wins(b1, a1, b2, a2):
    return b1 > b2 + 1e-9 or (abs(b1 - b2) <= 1e-9 and a1 is not None and
                             (a2 is None or a1 < a2))


robot = Robot()
dt = int(robot.getBasicTimeStep())
step_s = dt / 1000.0
name = robot.getName()
me = AGENTS.index(name)
mode_file = Path(__file__).resolve().parents[2] / "experiment_mode.txt"
duration_file = Path(__file__).resolve().parents[2] / "experiment_duration.txt"
mode = os.environ.get("CR_CBBA_MODE", "").strip().lower()
if not mode and mode_file.exists():
    mode = mode_file.read_text(encoding="utf-8").strip().lower()
if not mode:
    mode = "proposed"
mission_duration = (float(duration_file.read_text().strip())
                    if duration_file.exists() else MISSION_DURATION_S)
if mode not in ("baseline", "proposed"):
    raise ValueError("CR_CBBA_MODE must be baseline or proposed")

imu, gps, gyro = (robot.getDevice(n) for n in
                  ("inertial unit", "gps", "gyro"))
receiver, emitter = robot.getDevice("receiver"), robot.getDevice("emitter")
for device in (imu, gps, gyro, receiver): device.enable(dt)
receiver.setChannel(1); emitter.setChannel(1)
motors = [robot.getDevice(n) for n in ("front left propeller",
          "front right propeller", "rear left propeller", "rear right propeller")]
for motor in motors:
    motor.setPosition(float("inf")); motor.setVelocity(1)

out = Path(__file__).resolve().parents[2] / "results" / mode
out.mkdir(parents=True, exist_ok=True)
log_handle = (out / (name + ".jsonl")).open("w", encoding="utf-8")


def log(event, now, **data):
    data.update(time=round(now, 3), agent=name, mode=mode, event=event)
    log_handle.write(json.dumps(data, separators=(",", ":")) + "\n")
    log_handle.flush()


def send(kind, now, **data):
    data.update(kind=kind, time=now, agent=name)
    emitter.send(json.dumps(data, separators=(",", ":")).encode())


battery = INITIAL_SOC[me] * BATTERY_CAPACITY_J
minimum_soc = battery / BATTERY_CAPACITY_J
last_done = [-task[5] for task in TASKS]
winners, bids = [None] * len(TASKS), [0.0] * len(TASKS)
bundle, path = [], []
peers = {a: {"state": "UNKNOWN", "reservation": None, "active_task": None}
         for a in AGENTS}
reservation = None
state, active_task = "TAKEOFF", None
service_until = charge_until = wait_started = None
auction_until = 0.0
target = (0, 0, CRUISE_ALTITUDE)
last_alloc = last_status = -1e9
messages = reassignments = 0


def due(now):
    being_executed = {p["active_task"] for a, p in peers.items()
                      if a != name and p.get("active_task") is not None}
    return [j for j, task in enumerate(TASKS)
            if now - last_done[j] >= task[5] and j != active_task
            and j not in being_executed]


def reservations():
    result = {a: p["reservation"] for a, p in peers.items()
              if p["reservation"] is not None}
    if reservation is not None: result[name] = reservation
    return result


def rebuild(now, position):
    global bundle, path, reservation
    old = list(path)
    bundle = [j for j in bundle if winners[j] == name and j in due(now)]
    path = [j for j in path if j in bundle]
    while len(bundle) < BUNDLE_LIMIT:
        choices = []
        for j in due(now):
            if j in bundle: continue
            ins = best_insertion(position, path, j, now, battery, reservations(),
                                 name, mode == "proposed")
            if ins and wins(ins[0], name, bids[j], winners[j]):
                choices.append((ins[0], j, ins[1]))
        if not choices: break
        bid, j, index = max(choices, key=lambda q: (q[0], -q[1]))
        bundle.append(j); path.insert(index, j)
        bids[j], winners[j] = bid, name
    reservation = route_metrics(position, path, now)[3] if path else None
    if old != path:
        margin = battery - (route_metrics(position, path, now)[1] if path else
                            RESERVE_FRACTION * BATTERY_CAPACITY_J)
        log("bundle_update", now, path=[TASKS[j][0] for j in path],
            reservation=reservation, energy_margin_j=round(margin, 2))


def resolve(msg, now, position):
    global bundle, path, reservation, reassignments
    for j in range(len(TASKS)):
        if not wins(msg["bids"][j], msg["winners"][j], bids[j], winners[j]):
            continue
        if winners[j] == name and j in bundle:
            lost = bundle.index(j)
            for released in bundle[lost:]:
                if winners[released] == name: winners[released], bids[released] = None, 0.0
            bundle = bundle[:lost]; path = [k for k in path if k in bundle]
        winners[j], bids[j] = msg["winners"][j], msg["bids"][j]
    remote = msg.get("reservation")
    if mode != "proposed" or not overlaps(reservation, remote) or not bundle: return
    my_margin = battery - route_metrics(position, path, now)[1]
    if (my_margin, name) > (msg.get("margin", float("inf")), msg["agent"]):
        released = bundle.pop(); path.remove(released)
        winners[released], bids[released] = None, 0.0
        reservation = route_metrics(position, path, now)[3] if path else None
        reassignments += 1
        log("charger_conflict_resolved", now, with_agent=msg["agent"],
            released_task=TASKS[released][0], reservation=reservation)


def charge_or_wait(now):
    global state, charge_until, wait_started, reservation
    occupied = [a for a, p in peers.items() if a != name and p["state"] == "CHARGING"]
    if occupied:
        if mode == "proposed":
            latest = max((peers[a]["reservation"][1]
                          for a in occupied if peers[a]["reservation"]),
                         default=now + CHARGER_ACCESS_S)
            reservation = (latest, latest + CHARGER_ACCESS_S)
            state = "WAIT_RESERVATION"
            log("reservation_shift", now, after_agent=occupied[0],
                reservation=reservation)
            return
        state = "WAIT_CHARGER"
        if wait_started is None:
            wait_started = now; log("charger_conflict", now, occupied_by=occupied[0])
    else:
        if wait_started is not None:
            log("charger_wait_end", now, duration_s=round(now - wait_started, 3))
            wait_started = None
        state, charge_until = "CHARGING", now + CHARGER_ACCESS_S
        log("charge_start", now, until=charge_until)


def reserve_charger(now, position):
    """Choose the earliest non-overlapping access interval known locally."""
    arrival = now + distance(position, CHARGER) / CRUISE_SPEED
    candidate = (arrival, arrival + CHARGER_ACCESS_S)
    intervals = sorted(p["reservation"] for a, p in peers.items()
                       if a != name and p["reservation"] is not None)
    changed = True
    while changed:
        changed = False
        for interval in intervals:
            if overlaps(candidate, interval):
                candidate = (interval[1], interval[1] + CHARGER_ACCESS_S)
                changed = True
    return candidate


log("mission_start", 0, initial_soc=INITIAL_SOC[me])
while robot.step(dt) != -1:
    now = robot.getTime()
    if now > mission_duration:
        end_position = gps.getValues()
        log("mission_end", now, battery_soc=battery / BATTERY_CAPACITY_J,
            minimum_soc=minimum_soc,
            position=[end_position[0], end_position[1], end_position[2]],
            messages=messages, charging_reassignments=reassignments)
        log_handle.close(); break
    roll, pitch, yaw = imu.getRollPitchYaw()
    px, py, pz = gps.getValues(); position = (px, py)
    roll_rate, pitch_rate, _ = gyro.getValues()
    if state not in ("TAKEOFF", "CHARGING"):
        battery = max(0, battery - POWER_W * step_s)
        minimum_soc = min(minimum_soc, battery / BATTERY_CAPACITY_J)

    while receiver.getQueueLength():
        try: msg = json.loads(bytes(receiver.getBytes()).decode())
        finally: receiver.nextPacket()
        if msg.get("agent") == name: continue
        messages += 1
        if msg["kind"] == "status":
            peers[msg["agent"]].update(state=msg["state"],
                                        reservation=msg.get("reservation"),
                                        active_task=msg.get("active_task"))
            last_done = [max(a, b) for a, b in zip(last_done, msg["last_done"])]
        elif msg["kind"] == "allocation" and state not in ("TRANSIT", "TO_CHARGER"):
            resolve(msg, now, position)

    if now - last_alloc >= ALLOCATION_PERIOD_S and state in ("AUCTION", "IDLE", "WAIT_CHARGER"):
        started = time.perf_counter(); last_alloc = now; rebuild(now, position)
        margin = battery - (route_metrics(position, path, now)[1] if path else
                            RESERVE_FRACTION * BATTERY_CAPACITY_J)
        send("allocation", now, winners=winners, bids=bids, reservation=reservation,
             margin=margin)
        log("allocation_cycle", now,
            convergence_s=round(time.perf_counter() - started, 6))
    if now - last_status >= STATUS_PERIOD_S:
        last_status = now
        send("status", now, state=state, reservation=reservation,
             active_task=active_task, last_done=last_done)

    reserve = RESERVE_FRACTION * BATTERY_CAPACITY_J
    return_need = POWER_W * distance(position, CHARGER) / CRUISE_SPEED + reserve
    if state == "TAKEOFF":
        target = (px, py, CRUISE_ALTITUDE)
        if pz >= TAKEOFF_ALTITUDE_M:
            state, auction_until = "AUCTION", now + CONSENSUS_WINDOW_S
    elif state == "SERVICE":
        target = (px, py, CRUISE_ALTITUDE)
        if now >= service_until:
            last_done[active_task] = now
            log("task_complete", now, task=TASKS[active_task][0], task_type=TASKS[active_task][1])
            if active_task in bundle: bundle.remove(active_task)
            if active_task in path: path.remove(active_task)
            winners[active_task], bids[active_task] = None, 0.0
            active_task, state = None, "AUCTION"
            auction_until = now + CONSENSUS_WINDOW_S
    elif state == "CHARGING":
        target = (*CHARGER, CRUISE_ALTITUDE)
        # Status messages can reveal a same-step arrival after both UAVs
        # initially saw an idle dock. Deterministic arbitration keeps physical
        # capacity at one; this becomes measured waiting in the baseline.
        simultaneous = [a for a, p in peers.items()
                        if a < name and p["state"] == "CHARGING"]
        if simultaneous:
            state, wait_started = "WAIT_CHARGER", now
            log("charger_conflict", now, occupied_by=min(simultaneous))
        elif now >= charge_until:
            battery = BATTERY_CAPACITY_J
            log("charge_end", now); reservation = None; state = "AUCTION"
            auction_until = now + CONSENSUS_WINDOW_S
    elif state == "WAIT_CHARGER":
        angle = 2 * math.pi * me / len(AGENTS)
        target = (CHARGER[0] + HOLDING_RADIUS_M * math.cos(angle),
                  CHARGER[1] + HOLDING_RADIUS_M * math.sin(angle), CRUISE_ALTITUDE)
        if not any(p["state"] == "CHARGING" for a, p in peers.items() if a != name):
            charge_or_wait(now)
    elif state == "WAIT_RESERVATION":
        angle = 2 * math.pi * me / len(AGENTS)
        target = (CHARGER[0] + HOLDING_RADIUS_M * math.cos(angle),
                  CHARGER[1] + HOLDING_RADIUS_M * math.sin(angle), CRUISE_ALTITUDE)
        if reservation is not None and now >= reservation[0]:
            charge_or_wait(now)
    elif state == "TO_CHARGER":
        target = (*CHARGER, CRUISE_ALTITUDE)
        if distance(position, CHARGER) <= CHARGER_ARRIVAL_RADIUS_M:
            if mode == "proposed" and reservation is not None and now < reservation[0]:
                state = "WAIT_RESERVATION"
            else:
                charge_or_wait(now)
    elif state == "AUCTION" and now < auction_until:
        target = (px, py, CRUISE_ALTITUDE)
    elif (battery <= max(return_need + POWER_W * 10,
                         .32 * BATTERY_CAPACITY_J) or
          (not path and battery < .35 * BATTERY_CAPACITY_J)):
        if mode == "proposed":
            reservation = reserve_charger(now, position)
        state = "TO_CHARGER"; target = (*CHARGER, CRUISE_ALTITUDE)
        log("charger_request", now, reservation=reservation, position=position,
            battery_soc=battery / BATTERY_CAPACITY_J)
    elif path:
        active_task = path[0]; task = TASKS[active_task]
        state = "TRANSIT"; target = (task[2], task[3], CRUISE_ALTITUDE)
        if distance(position, task[2:4]) <= ARRIVAL_RADIUS_M:
            state, service_until = "SERVICE", now + task[6]
            log("task_start", now, task=task[0], duration_s=task[6])
    else:
        state = "AUCTION"; auction_until = now + CONSENSUS_WINDOW_S
        target = (px, py, CRUISE_ALTITUDE)

    dx, dy = target[0] - px, target[1] - py
    vx, vy, _ = gps.getSpeedVector()
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    left = -math.sin(yaw) * dx + math.cos(yaw) * dy
    forward_speed = math.cos(yaw) * vx + math.sin(yaw) * vy
    left_speed = -math.sin(yaw) * vx + math.cos(yaw) * vy
    # Prioritize altitude recovery and use conservative horizontal commands;
    # this prevents the stock Mavic dynamics from losing lift on long flights.
    horizontal_scale = 1.0 if pz > 10.5 else 0.0
    ri = (50 * clamp(roll, -1, 1) + roll_rate +
          horizontal_scale * clamp(.15 * left - .4 * left_speed, -1, 1))
    pi = (30 * clamp(pitch, -1, 1) + pitch_rate +
          horizontal_scale * clamp(-.15 * forward + .4 * forward_speed, -1, 1))
    base = 68.5 + 3 * clamp(target[2] - pz + .6, -1, 1) ** 3
    motors[0].setVelocity(base - ri + pi); motors[1].setVelocity(-(base + ri + pi))
    motors[2].setVelocity(-(base - ri - pi)); motors[3].setVelocity(base + ri - pi)
