"""Standard CBBA with isolated energy and shared-charger constraints."""
import json
import math
import os
from pathlib import Path

from controller import Supervisor
from mission_config import *

EPSILON = 1e-9
N_TASKS = len(TASKS)


def clamp(value, low, high):
    return max(low, min(value, high))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wins(bid_a, agent_a, bid_b, agent_b):
    return (bid_a > bid_b + EPSILON or
            (abs(bid_a - bid_b) <= EPSILON and agent_a is not None and
             (agent_b is None or agent_a < agent_b)))


def overlaps(interval_a, interval_b):
    return (interval_a is not None and interval_b is not None and
            interval_a[0] < interval_b[1] and interval_b[0] < interval_a[1])


robot = Supervisor()
dt = int(robot.getBasicTimeStep())
step_seconds = dt / 1000.0
name = robot.getName()
me = AGENTS.index(name)
project = Path(__file__).resolve().parents[2]
mode = os.environ.get("CR_CBBA_MODE", "").strip().lower()
if not mode:
    mode = (project / "experiment_mode.txt").read_text().strip().lower()
if mode not in ("baseline", "proposed"):
    raise ValueError("experiment mode must be baseline or proposed")

gps = robot.getDevice("gps")
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
for device in (gps, receiver):
    device.enable(dt)
receiver.setChannel(1)
emitter.setChannel(1)
self_node = robot.getSelf()
translation = self_node.getField("translation")

output = project / "results" / mode
output.mkdir(parents=True, exist_ok=True)
log_file = (output / f"{name}.jsonl").open("w", encoding="utf-8")


def log(event, now, **fields):
    fields.update(event=event, time=round(now, 3), agent=name, mode=mode)
    log_file.write(json.dumps(fields, separators=(",", ":")) + "\n")
    log_file.flush()


def transmit(kind, now, **fields):
    global messages_sent
    fields.update(kind=kind, time=now, agent=me)
    emitter.send(json.dumps(fields, separators=(",", ":")).encode())
    messages_sent += 1


# Standard CBBA local information: bundle b_i, path p_i, winning bid y_i,
# winner z_i, and timestamp vector s_i (Choi, Brunet, and How, 2009).
bundle = []
path = []
y = [0.0] * N_TASKS
z = [None] * N_TASKS
s = [0.0] * len(AGENTS)

battery = INITIAL_SOC[me] * BATTERY_CAPACITY_J
minimum_soc = battery / BATTERY_CAPACITY_J
last_started = [None] * N_TASKS
peers = [{"state": "UNKNOWN", "active": None, "bundle": [],
          "reservation": None, "bundle_committed": False,
          "reservation_committed": False, "margin": float("inf")}
         for _ in AGENTS]
reservation = None
reservation_path = ()
bundle_committed = False
reservation_committed = False
charge_after_bundle = False
state = "TAKEOFF"
active_task = None
service_until = None
charge_until = None
wait_started = None
target = (0.0, 0.0, CRUISE_ALTITUDE)
holding_point = CHARGER
last_status_payload = None
auction_time = 0.0
auction_rounds = 0
quiet_rounds = 0
changed = False
messages_sent = 0
reassignments = 0
charging_releases = set()


def route_metrics(position, candidate_path, now):
    point = position
    elapsed = 0.0
    score = 0.0
    for task_id in candidate_path:
        task = TASKS[task_id]
        elapsed += distance(point, task[2:4]) / CRUISE_SPEED + task[6]
        score += task[4] * math.exp(-LAMBDA * elapsed / 60.0)
        point = task[2:4]
    elapsed += distance(point, CHARGER) / CRUISE_SPEED
    arrival = now + elapsed
    contingency = ((len(AGENTS) - 1) * CHARGER_ACCESS_S
                   if mode == "baseline" else 0.0)
    required_energy = (POWER_W * (elapsed + contingency) +
                       RESERVE_FRACTION * BATTERY_CAPACITY_J)
    return score, required_energy, arrival, (arrival, arrival + CHARGER_ACCESS_S)


def reservation_energy(position, candidate_path, now, interval):
    """Route energy including any hover until an already chosen reservation."""
    _, required, arrival, _ = route_metrics(position, candidate_path, now)
    waiting = max(0.0, interval[0] - arrival) if interval else 0.0
    return required + POWER_W * waiting


def route_end(position, candidate_path):
    point = position
    elapsed = 0.0
    for task_id in candidate_path:
        task = TASKS[task_id]
        elapsed += distance(point, task[2:4]) / CRUISE_SPEED + task[6]
        point = task[2:4]
    return point, elapsed


def can_service_another(position, available_energy):
    contingency = ((len(AGENTS) - 1) * CHARGER_ACCESS_S
                   if mode == "baseline" else MAX_SLOT_DELAY_S)
    reserve = (RESERVE_FRACTION * BATTERY_CAPACITY_J +
               POWER_W * contingency)
    return any(POWER_W * (distance(position, task[2:4]) / CRUISE_SPEED +
                          task[6] + distance(task[2:4], CHARGER) /
                          CRUISE_SPEED) + reserve <= available_energy
               for task in TASKS)


def needs_charge_after(position, candidate_path):
    endpoint, elapsed = route_end(position, candidate_path)
    return not can_service_another(endpoint, battery - POWER_W * elapsed)


def planned_reservation(position, candidate_path, now):
    if mode != "proposed" or not candidate_path or not needs_charge_after(
            position, candidate_path):
        return None
    return route_metrics(position, candidate_path, now)[3]


def starts_on_time(position, candidate_path, now):
    point = position
    elapsed = 0.0
    for task_id in candidate_path:
        task = TASKS[task_id]
        elapsed += distance(point, task[2:4]) / CRUISE_SPEED
        if now <= deadline(task_id) < now + elapsed:
            return False
        elapsed += task[6]
        point = task[2:4]
    return True


def release_time(task_id):
    if last_started[task_id] is None:
        return 0.0
    return deadline(task_id) - EARLY_RELEASE_S


def deadline(task_id):
    started = last_started[task_id]
    return TASKS[task_id][5] if started is None else started + TASKS[task_id][5]


def due_tasks(now):
    executing = {peer["active"] for index, peer in enumerate(peers)
                 if index != me and peer["active"] is not None}
    committed = {task_id for index, peer in enumerate(peers)
                 if index != me and peer["bundle_committed"]
                 for task_id in peer["bundle"]}
    return [task_id for task_id, task in enumerate(TASKS)
            if now >= release_time(task_id) and task_id != active_task
            and task_id not in executing and task_id not in committed]


def best_insertion(position, task_id, now):
    base_score = route_metrics(position, path, now)[0]
    choices = []
    for index in range(len(path) + 1):
        candidate = path[:index] + [task_id] + path[index:]
        if not starts_on_time(position, candidate, now):
            continue
        score, energy, _, _ = route_metrics(position, candidate, now)
        if energy > battery:
            continue
        preferred = planned_reservation(position, candidate, now)
        if preferred is not None and feasible_reservation(
                position, candidate, now, preferred) is None:
            continue
        choices.append((score - base_score, index))
    return max(choices, default=None, key=lambda item: (item[0], -item[1]))


def build_bundle(now, position):
    """Standard CBBA phase 1: sequential greedy bundle construction."""
    global changed, reservation, reservation_path
    while len(bundle) < BUNDLE_LIMIT:
        candidates = []
        for task_id in due_tasks(now):
            if task_id in bundle or task_id in charging_releases:
                continue
            insertion = best_insertion(position, task_id, now)
            if insertion and wins(insertion[0], me, y[task_id], z[task_id]):
                candidates.append((insertion[0], task_id, insertion[1]))
        if not candidates:
            break
        bid, task_id, index = max(candidates,
                                  key=lambda item: (item[0], -item[1]))
        bundle.append(task_id)
        path.insert(index, task_id)
        y[task_id], z[task_id] = bid, me
        changed = True
    if tuple(path) != reservation_path:
        preferred = planned_reservation(position, path, now)
        updated = (feasible_reservation(position, path, now, preferred)
                   if preferred is not None else None)
        reservation_path = tuple(path)
        if updated != reservation:
            reservation = updated
            changed = True


def truncate_lost_bundle():
    """Remove a lost task and every task added after it (standard CBBA rule)."""
    global bundle, path, changed
    lost = next((index for index, task_id in enumerate(bundle)
                 if z[task_id] != me), None)
    if lost is None:
        return
    for task_id in bundle[lost + 1:]:
        if z[task_id] == me:
            y[task_id], z[task_id] = 0.0, None
    bundle = bundle[:lost]
    path = [task_id for task_id in path if task_id in bundle]
    changed = True


def resolve_cbba(message, received_at):
    """Standard CBBA phase 2 consensus table, including timestamp rules."""
    global changed
    sender = message["agent"]
    sender_y, sender_z, sender_s = message["y"], message["z"], message["s"]
    for task_id in range(N_TASKS):
        if task_id == active_task:
            continue                    # service already in execution is firm
        remote_winner = sender_z[task_id]
        local_winner = z[task_id]
        action = "leave"
        if remote_winner == sender:
            if local_winner == me:
                action = "update" if wins(sender_y[task_id], sender,
                                            y[task_id], me) else "leave"
            elif local_winner in (sender, None):
                action = "update"
            else:
                action = ("update" if sender_s[local_winner] > s[local_winner]
                          or wins(sender_y[task_id], sender,
                                  y[task_id], local_winner) else "leave")
        elif remote_winner == me:
            if local_winner == sender:
                action = "reset"
            elif (local_winner not in (me, None) and
                  sender_s[local_winner] > s[local_winner]):
                action = "reset"
        elif remote_winner is not None:
            if local_winner == me:
                action = ("update" if sender_s[remote_winner] > s[remote_winner]
                          and wins(sender_y[task_id], remote_winner,
                                   y[task_id], me) else "leave")
            elif local_winner == sender:
                action = ("update" if sender_s[remote_winner] > s[remote_winner]
                          else "reset")
            elif local_winner == remote_winner:
                action = ("update" if sender_s[remote_winner] > s[remote_winner]
                          else "leave")
            elif local_winner is None:
                action = ("update" if sender_s[remote_winner] > s[remote_winner]
                          else "leave")
            else:
                if (sender_s[remote_winner] > s[remote_winner] and
                    (sender_s[local_winner] > s[local_winner] or
                     wins(sender_y[task_id], remote_winner,
                          y[task_id], local_winner))):
                    action = "update"
                elif (sender_s[local_winner] > s[local_winner] and
                      s[remote_winner] > sender_s[remote_winner]):
                    action = "reset"
        elif (local_winner == sender or
              (local_winner not in (me, None) and
               sender_s[local_winner] > s[local_winner])):
            action = "update"

        if action == "update" and (y[task_id], z[task_id]) != (
                sender_y[task_id], remote_winner):
            y[task_id], z[task_id] = sender_y[task_id], remote_winner
            changed = True
        elif action == "reset" and (y[task_id], z[task_id]) != (0.0, None):
            y[task_id], z[task_id] = 0.0, None
            changed = True

    for agent in range(len(AGENTS)):
        if agent not in (me, sender):
            s[agent] = max(s[agent], sender_s[agent])
    s[sender] = received_at


def feasible_reservation(position, candidate_path, now, interval):
    arrival = route_metrics(position, candidate_path, now)[2]
    for attempt in range(len(AGENTS)):
        if interval is not None and interval[0] - arrival > MAX_SLOT_DELAY_S:
            return None
        margin = battery - reservation_energy(
            position, candidate_path, now, interval)
        conflicts = [peer["reservation"] for index, peer in enumerate(peers)
                     if index != me and peer["reservation"] is not None
                     and (peer["reservation_committed"] or
                          (peer["margin"], index) < (margin, me))
                     and overlaps(interval, peer["reservation"])]
        if not conflicts:
            return (interval if interval is None or reservation_energy(
                    position, candidate_path, now, interval) <= battery else None)
        if attempt == len(AGENTS) - 1:
            return None
        start = max(other[1] for other in conflicts)
        interval = (start, start + CHARGER_ACCESS_S)


def next_open_reservation(interval):
    intervals = [peer["reservation"] for index, peer in enumerate(peers)
                 if index != me and peer["reservation"] is not None]
    for _ in range(len(AGENTS)):
        conflicts = [other for other in intervals if overlaps(interval, other)]
        if not conflicts:
            return interval
        start = max(other[1] for other in conflicts)
        interval = (start, start + CHARGER_ACCESS_S)


def resolve_charger(message, now, position):
    global reservation, reservation_path, reassignments, changed
    remote = message.get("reservation")
    remote_committed = message.get(
        "reservation_committed",
        peers[message["agent"]]["reservation_committed"])
    if (mode != "proposed" or reservation_committed or
            not overlaps(reservation, remote)):
        return
    my_margin = battery - reservation_energy(
        position, path, now, reservation)
    remote_margin = message.get("margin", float("inf"))
    sender = message["agent"]
    if remote_committed or (my_margin, me) > (remote_margin, sender):
        released_tasks = []
        candidate = feasible_reservation(position, path, now, reservation)
        if candidate is not None:
            if candidate != reservation:
                reservation = candidate
                reservation_path = tuple(path)
                changed = True
                log("charger_slot_shifted", now, with_agent=AGENTS[sender],
                    reservation=reservation)
            return
        while bundle:
            released = bundle.pop()
            path.remove(released)
            y[released], z[released] = 0.0, None
            charging_releases.add(released)
            released_tasks.append(TASKS[released][0])
            candidate = planned_reservation(position, path, now)
            candidate = feasible_reservation(position, path, now, candidate)
            if candidate is not None or not needs_charge_after(position, path):
                reservation = candidate
                reservation_path = tuple(path)
                break
        if released_tasks:
            if not bundle:
                reservation = None
                reservation_path = ()
            reassignments += len(released_tasks)
            changed = True
            log("charger_conflict_resolved", now,
                with_agent=AGENTS[sender],
                released_tasks=released_tasks,
                released_count=len(released_tasks),
                reservation=reservation)


def enter_auction():
    global state, auction_time, auction_rounds, quiet_rounds, changed
    global bundle_committed, reservation_committed, charge_after_bundle
    state = "AUCTION"
    auction_time = robot.getTime()
    auction_rounds = 0
    quiet_rounds = 0
    changed = True
    bundle_committed = False
    reservation_committed = False
    charge_after_bundle = False
    charging_releases.clear()


def release_bundle(clear_reservation=True):
    global bundle, path, reservation, reservation_path
    global bundle_committed, charge_after_bundle
    for task_id in bundle:
        if z[task_id] == me:
            y[task_id], z[task_id] = 0.0, None
    bundle, path = [], []
    bundle_committed = False
    charge_after_bundle = False
    if clear_reservation:
        reservation = None
        reservation_path = ()


def request_charger(now, position):
    """Execute the charger leg already committed during allocation."""
    global state, reservation, reservation_committed, holding_point
    release_bundle(clear_reservation=False)
    if reservation is None and mode == "proposed":
        preferred = route_metrics(position, [], now)[3]
        reservation = (feasible_reservation(position, [], now, preferred) or
                       next_open_reservation(preferred))
        reservation_committed = reservation is not None
    approach = distance(position, CHARGER)
    if approach > EPSILON:
        scale = HOLDING_DISTANCE_M / approach
        holding_point = (CHARGER[0] + (position[0] - CHARGER[0]) * scale,
                         CHARGER[1] + (position[1] - CHARGER[1]) * scale)
    state = "TO_CHARGER"
    log("charger_request", now, reservation=reservation,
        battery_soc=battery / BATTERY_CAPACITY_J)


def begin_charging(now):
    global state, wait_started
    occupied = [index for index, peer in enumerate(peers)
                if index != me and peer["state"] == "CHARGING"]
    if occupied:
        if mode == "baseline":
            state = "WAIT_CHARGER"
            if wait_started is None:
                wait_started = now
                log("charger_conflict", now, occupied_by=AGENTS[occupied[0]])
        else:
            # A committed proposed reservation should make this unreachable.
            state = "WAIT_RESERVATION"
        return
    state = "DOCKING"


log("mission_start", 0.0, initial_soc=INITIAL_SOC[me])
while robot.step(dt) != -1:
    now = robot.getTime()
    if now > MISSION_DURATION_S:
        log("mission_end", now, minimum_soc=minimum_soc, messages=messages_sent,
            charging_reassignments=reassignments)
        log_file.close()
        break

    px, py, pz = gps.getValues()
    position = (px, py)
    allocation_reply = False
    if state not in ("TAKEOFF", "CHARGING"):
        battery = max(0.0, battery - POWER_W * step_seconds)
        minimum_soc = min(minimum_soc, battery / BATTERY_CAPACITY_J)

    while receiver.getQueueLength():
        try:
            message = json.loads(bytes(receiver.getBytes()).decode())
        finally:
            receiver.nextPacket()
        if message.get("agent") == me:
            continue
        sender = message["agent"]
        if message["kind"] == "status":
            peers[sender].update(state=message["state"],
                                 active=message.get("active"),
                                 bundle=message.get("bundle", []),
                                 reservation=message.get("reservation"),
                                 bundle_committed=message.get(
                                     "bundle_committed", False),
                                 reservation_committed=message.get(
                                     "reservation_committed", False))
            for task_id, started_at in enumerate(message["last_started"]):
                if (started_at is not None and
                        (last_started[task_id] is None or
                         started_at > last_started[task_id])):
                    last_started[task_id] = started_at
                    if task_id != active_task:
                        y[task_id], z[task_id] = 0.0, None
                        if task_id in bundle:
                            bundle.remove(task_id)
                            changed = True
                        if task_id in path:
                            path.remove(task_id)
        elif message["kind"] == "allocation":
            allocation_reply |= message.get("request_reply", False)
            peers[sender]["reservation"] = message.get("reservation")
            peers[sender]["reservation_committed"] = message.get(
                "reservation_committed", False)
            peers[sender]["margin"] = message.get("margin", float("inf"))
            resolve_cbba(message, now)
            if state == "AUCTION":
                resolve_charger(message, auction_time, position)

    if state == "AUCTION":
        truncate_lost_bundle()
        build_bundle(auction_time, position)
        auction_rounds += 1
        quiet_rounds = 0 if changed else quiet_rounds + 1
        changed = False
        if quiet_rounds >= QUIET_ROUNDS:
            bundle_committed = bool(bundle)
            charge_after_bundle = needs_charge_after(position, path)
            reservation_committed = reservation is not None
            log("allocation_converged", now,
                rounds=auction_rounds,
                path=[TASKS[task_id][0] for task_id in path],
                reservation=reservation)
            state = "IDLE"

    if state == "AUCTION" or allocation_reply:
        margin = (battery - reservation_energy(
                      position, path, auction_time, reservation) if reservation
                  else battery - RESERVE_FRACTION * BATTERY_CAPACITY_J)
        s[me] = now
        transmit("allocation", now, y=y, z=z, s=s,
                 reservation=reservation, margin=margin,
                 reservation_committed=reservation_committed,
                 request_reply=state == "AUCTION")

    if state == "TAKEOFF":
        target = (px, py, CRUISE_ALTITUDE)
        if pz >= TAKEOFF_ALTITUDE_M:
            enter_auction()
    elif state == "SERVICE":
        target = (px, py, CRUISE_ALTITUDE)
        if now >= service_until:
            log("task_complete", now, task=TASKS[active_task][0],
                task_type=TASKS[active_task][1])
            if active_task in bundle:
                bundle.remove(active_task)
            if active_task in path:
                path.remove(active_task)
            y[active_task], z[active_task] = 0.0, None
            active_task = None
            if not bundle:
                bundle_committed = False
            state = "IDLE"
    elif state == "TRANSIT":
        task = TASKS[active_task]
        target = (task[2], task[3], CRUISE_ALTITUDE)
        if distance(position, task[2:4]) <= ARRIVAL_RADIUS_M:
            task_deadline = deadline(active_task)
            last_started[active_task] = now
            state = "SERVICE"
            service_until = now + task[6]
            log("task_start", now, task=task[0], duration_s=task[6],
                deadline=task_deadline, late=now > task_deadline)
    elif state == "IDLE":
        if path:
            task_id = path[0]
            task = TASKS[task_id]
            task_energy = route_metrics(position, [task_id], now)[1]
            if task_energy > battery:
                request_charger(now, position)
            else:
                active_task = task_id
                state = "TRANSIT"
                target = (task[2], task[3], CRUISE_ALTITUDE)
        elif charge_after_bundle or not can_service_another(position, battery):
            request_charger(now, position)
        elif any(z[task_id] in (None, me) for task_id in due_tasks(now)):
            enter_auction()
    elif state == "TO_CHARGER":
        target = (*CHARGER, CRUISE_ALTITUDE)
        charger_distance = distance(position, CHARGER)
        arrival = now + charger_distance / CRUISE_SPEED
        if (mode == "proposed" and reservation is not None and
                arrival >= reservation[1]):
            missed = reservation
            reservation_committed = False
            preferred = (arrival, arrival + CHARGER_ACCESS_S)
            reservation = (feasible_reservation(
                position, [], now, preferred) or
                next_open_reservation(preferred))
            reservation_committed = True
            log("reservation_missed", now, previous=missed,
                reservation=reservation)
        occupied = any(peer["state"] in ("DOCKING", "CHARGING")
                       for index, peer in enumerate(peers) if index != me)
        early = (mode == "proposed" and reservation and
                 now + charger_distance / CRUISE_SPEED < reservation[0])
        if charger_distance <= HOLDING_DISTANCE_M and (occupied or early):
            state = "WAIT_RESERVATION" if mode == "proposed" else "WAIT_CHARGER"
            wait_started = now
            if occupied and mode == "baseline":
                log("charger_conflict", now)
            target = (*holding_point, CRUISE_ALTITUDE)
        elif charger_distance <= CHARGER_ARRIVAL_RADIUS_M:
            begin_charging(now)
    elif state in ("WAIT_CHARGER", "WAIT_RESERVATION"):
        target = (*holding_point, CRUISE_ALTITUDE)
        travel_time = distance(position, CHARGER) / CRUISE_SPEED
        ready = (state == "WAIT_CHARGER" or reservation is None or
                 now + travel_time >= reservation[0])
        occupied = any(peer["state"] in ("DOCKING", "CHARGING")
                       for index, peer in enumerate(peers) if index != me)
        if ready and not occupied:
            if wait_started is not None:
                log("charger_wait_end", now, duration_s=now - wait_started)
                wait_started = None
            state = "TO_CHARGER"
            target = (*CHARGER, CRUISE_ALTITUDE)
    elif state == "DOCKING":
        target = (*CHARGER, CRUISE_ALTITUDE)
        simultaneous = [index for index, peer in enumerate(peers)
                        if index < me and peer["state"] in
                        ("DOCKING", "CHARGING")]
        if simultaneous:
            state = "WAIT_RESERVATION" if mode == "proposed" else "WAIT_CHARGER"
            wait_started = now
            log("charger_conflict", now,
                occupied_by=AGENTS[min(simultaneous)])
        else:
            battery = BATTERY_CAPACITY_J
            state = "CHARGING"
            charge_until = now + CHARGER_ACCESS_S
            reservation = (now, charge_until)
            log("charge_start", now, until=charge_until)
    elif state == "CHARGING":
        target = (*CHARGER, CRUISE_ALTITUDE)
        if now >= charge_until:
            log("charge_end", now)
            reservation = None
            enter_auction()
    else:  # AUCTION
        target = (px, py, CRUISE_ALTITUDE)

    status_payload = dict(state=state, active=active_task, bundle=bundle,
                          reservation=reservation,
                          bundle_committed=bundle_committed,
                          reservation_committed=reservation_committed,
                          last_started=last_started)
    status_signature = json.dumps(status_payload, sort_keys=True)
    if status_signature != last_status_payload:
        last_status_payload = status_signature
        transmit("status", now, **status_payload)

    delta = (target[0] - px, target[1] - py, target[2] - pz)
    remaining = math.sqrt(sum(component * component for component in delta))
    step_length = min(CRUISE_SPEED * step_seconds, remaining)
    if remaining > EPSILON:
        scale = step_length / remaining
        translation.setSFVec3f([px + scale * delta[0],
                                py + scale * delta[1],
                                pz + scale * delta[2]])
