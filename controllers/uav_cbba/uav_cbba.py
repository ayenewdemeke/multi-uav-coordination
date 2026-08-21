"""Standard CBBA with isolated energy and shared-charger constraints."""
import json
import math
import os
from pathlib import Path

from controller import Supervisor
from mission_config import *

EPSILON = 1e-9
NETWORK_DIAMETER = 1                 # all UAVs broadcast to all other UAVs
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
self_node.getField("physics").removeSF()  # task-level kinematic experiment

output = project / "results" / mode
output.mkdir(parents=True, exist_ok=True)
log_file = (output / f"{name}.jsonl").open("w", encoding="utf-8")


def log(event, now, **fields):
    fields.update(event=event, time=round(now, 3), agent=name, mode=mode)
    log_file.write(json.dumps(fields, separators=(",", ":")) + "\n")
    log_file.flush()


def transmit(kind, now, **fields):
    fields.update(kind=kind, time=now, agent=me)
    emitter.send(json.dumps(fields, separators=(",", ":")).encode())


# Standard CBBA local information: bundle b_i, path p_i, winning bid y_i,
# winner z_i, and timestamp vector s_i (Choi, Brunet, and How, 2009).
bundle = []
path = []
y = [0.0] * N_TASKS
z = [None] * N_TASKS
s = [0.0] * len(AGENTS)

battery = INITIAL_SOC[me] * BATTERY_CAPACITY_J
minimum_soc = battery / BATTERY_CAPACITY_J
last_completed = [-task[5] for task in TASKS]
peers = [{"state": "UNKNOWN", "active": None, "bundle": [],
          "reservation": None, "committed": False}
         for _ in AGENTS]
reservation = None
reservation_committed = False
state = "TAKEOFF"
active_task = None
service_until = None
charge_until = None
wait_started = None
target = (0.0, 0.0, CRUISE_ALTITUDE)
last_auction_step = -1e9
last_status = -1e9
auction_started = 0.0
auction_rounds = 0
quiet_rounds = 0
changed = False
messages = 0
reassignments = 0


def route_metrics(position, candidate_path, now, wait_for=None):
    """Return route score, required energy, charger arrival, and reservation.

    ``wait_for`` is used only when a UAV has no task left to release after a
    charger conflict.  The resulting holding time is part of route energy.
    """
    point = position
    elapsed = 0.0
    score = 0.0
    for task_id in candidate_path:
        task = TASKS[task_id]
        elapsed += distance(point, task[2:4]) / CRUISE_SPEED + task[6]
        score += task[4] * math.exp(-LAMBDA * elapsed)
        point = task[2:4]
    elapsed += distance(point, CHARGER) / CRUISE_SPEED
    arrival = now + elapsed
    start = arrival
    for other in sorted(wait_for or []):
        if overlaps((start, start + CHARGER_ACCESS_S), other):
            start = other[1]
    waiting = max(0.0, start - arrival)
    required_energy = (POWER_W * (elapsed + waiting) +
                       RESERVE_FRACTION * BATTERY_CAPACITY_J)
    return score, required_energy, arrival, (start, start + CHARGER_ACCESS_S)


def reservation_energy(position, candidate_path, now, interval):
    """Route energy including any hover until an already chosen reservation."""
    _, required, arrival, _ = route_metrics(position, candidate_path, now)
    waiting = max(0.0, interval[0] - arrival) if interval else 0.0
    return required + POWER_W * waiting


def release_time(task_id):
    """Earliest replanning time that still permits an on-time revisit.

    The lead time is derived from the task service time and the longest
    possible inbound trip from another task or the charger.  It is therefore
    not an experimental tuning parameter.
    """
    completed = last_completed[task_id]
    if completed <= 0.0:              # every task is initially available
        return 0.0
    task = TASKS[task_id]
    origins = [other[2:4] for other in TASKS] + [CHARGER]
    inbound = max(distance(origin, task[2:4]) for origin in origins)
    return completed + task[5] - task[6] - inbound / CRUISE_SPEED


def deadline(task_id):
    completed = last_completed[task_id]
    return TASKS[task_id][5] if completed <= 0.0 else completed + TASKS[task_id][5]


def due_tasks(now):
    executing = {peer["active"] for index, peer in enumerate(peers)
                 if index != me and peer["active"] is not None}
    committed = {task_id for index, peer in enumerate(peers)
                 if index != me and peer["committed"]
                 for task_id in peer["bundle"]}
    return [task_id for task_id, task in enumerate(TASKS)
            if now >= release_time(task_id) and task_id != active_task
            and task_id not in executing and task_id not in committed]


def peer_reservations(committed_only=False):
    return {index: peer["reservation"] for index, peer in enumerate(peers)
            if index != me and peer["reservation"] is not None
            and (not committed_only or peer["committed"])}


def best_insertion(position, task_id, now):
    base_score = route_metrics(position, path, now)[0]
    choices = []
    for index in range(len(path) + 1):
        candidate = path[:index] + [task_id] + path[index:]
        score, energy, _, interval = route_metrics(position, candidate, now)
        if energy > battery:
            continue
        if mode == "proposed" and any(
                overlaps(interval, other)
                for other in peer_reservations(committed_only=True).values()):
            continue
        choices.append((score - base_score, index))
    return max(choices, default=None, key=lambda item: (item[0], -item[1]))


def build_bundle(now, position):
    """Standard CBBA phase 1: sequential greedy bundle construction."""
    global changed, reservation
    while len(bundle) < BUNDLE_LIMIT:
        candidates = []
        for task_id in due_tasks(now):
            if task_id in bundle:
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
    reservation = route_metrics(position, path, now)[3] if path else reservation


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


def resolve_charger(message, now, position):
    """Proposed extension applied after the standard CBBA consensus step."""
    global reservation, reassignments, changed
    remote = message.get("reservation")
    remote_committed = message.get(
        "reservation_committed", peers[message["agent"]]["committed"])
    if (mode != "proposed" or reservation_committed or
            not overlaps(reservation, remote)):
        return
    my_margin = battery - reservation_energy(
        position, path, now, reservation)
    remote_margin = message.get("margin", float("inf"))
    sender = message["agent"]
    # A reservation already accepted at convergence is firm. Otherwise the
    # normal smaller-energy-margin rule arbitrates two tentative intervals.
    if remote_committed or (my_margin, me) > (remote_margin, sender):
        released_tasks = []
        while bundle:
            _, required_energy, _, reservation = route_metrics(
                position, path, now)
            if required_energy <= battery and not overlaps(reservation, remote):
                break
            released = bundle.pop()
            path.remove(released)
            y[released], z[released] = 0.0, None
            released_tasks.append(TASKS[released][0])
            reservation = (route_metrics(position, path, now)[3]
                           if path else None)
        if not bundle:
            _, _, _, direct = route_metrics(position, [], now)
            wait_for = [remote] if overlaps(direct, remote) else []
            _, required_energy, _, reservation = route_metrics(
                position, [], now, wait_for)
            if required_energy > battery:
                reservation = None
        if released_tasks:
            reassignments += len(released_tasks)
            changed = True
            log("charger_conflict_resolved", now,
                with_agent=AGENTS[sender],
                released_tasks=released_tasks,
                released_count=len(released_tasks),
                reservation=reservation)


def enter_auction(now):
    global state, auction_started, auction_rounds, quiet_rounds, changed
    global reservation_committed
    state = "AUCTION"
    auction_started = now
    auction_rounds = 0
    quiet_rounds = 0
    changed = True
    reservation_committed = False


def release_bundle(clear_reservation=True):
    global bundle, path, reservation
    for task_id in bundle:
        if z[task_id] == me:
            y[task_id], z[task_id] = 0.0, None
    bundle, path = [], []
    if clear_reservation:
        reservation = None


def request_charger(now, position):
    """Execute the charger leg already committed during allocation."""
    global state, reservation, reservation_committed
    release_bundle(clear_reservation=False)
    if reservation is None:  # emergency fallback; never the normal route
        wait_for = peer_reservations().values() if mode == "proposed" else []
        reservation = route_metrics(position, [], now, wait_for)[3]
        reservation_committed = True
    state = "TO_CHARGER"
    log("charger_request", now, reservation=reservation,
        battery_soc=battery / BATTERY_CAPACITY_J)


def begin_charging(now):
    global state, charge_until, wait_started, reservation
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
    if wait_started is not None:
        log("charger_wait_end", now, duration_s=now - wait_started)
        wait_started = None
    state = "CHARGING"
    if mode == "proposed" and reservation is not None:
        if now >= reservation[1]:
            log("reservation_missed", now, reservation=reservation)
            reservation = (now, now + CHARGER_ACCESS_S)
        charge_until = reservation[1]
    else:
        charge_until = now + CHARGER_ACCESS_S
        reservation = (now, charge_until)
    log("charge_start", now, until=charge_until)


log("mission_start", 0.0, initial_soc=INITIAL_SOC[me])
while robot.step(dt) != -1:
    now = robot.getTime()
    if now > MISSION_DURATION_S:
        log("mission_end", now, minimum_soc=minimum_soc, messages=messages,
            charging_reassignments=reassignments)
        log_file.close()
        break

    px, py, pz = gps.getValues()
    position = (px, py)
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
        messages += 1
        sender = message["agent"]
        if message["kind"] == "status":
            peers[sender].update(state=message["state"],
                                 active=message.get("active"),
                                 bundle=message.get("bundle", []),
                                 reservation=message.get("reservation"),
                                 committed=message.get("committed", False))
            for task_id, completed_at in enumerate(message["last_completed"]):
                if completed_at > last_completed[task_id]:
                    last_completed[task_id] = completed_at
                    if task_id != active_task:
                        y[task_id], z[task_id] = 0.0, None
                        if task_id in bundle:
                            bundle.remove(task_id)
                        if task_id in path:
                            path.remove(task_id)
        elif message["kind"] == "allocation":
            resolve_cbba(message, now)
            if state == "AUCTION":
                resolve_charger(message, now, position)

    if now - last_status >= STATUS_PERIOD_S:
        last_status = now
        transmit("status", now, state=state, active=active_task,
                 bundle=bundle, reservation=reservation,
                 committed=reservation_committed,
                 last_completed=last_completed)

    # Winner/bid/timestamp information remains live in every flight state.
    if now - last_auction_step >= AUCTION_PERIOD_S:
        last_auction_step = now
        margin = (battery - reservation_energy(
                      position, path, now, reservation) if reservation
                  else battery - RESERVE_FRACTION * BATTERY_CAPACITY_J)
        s[me] = now
        transmit("allocation", now, y=y, z=z, s=s,
                 reservation=reservation, margin=margin,
                 reservation_committed=reservation_committed)

    if state == "AUCTION" and now - auction_started >= auction_rounds * AUCTION_PERIOD_S:
        truncate_lost_bundle()
        build_bundle(now, position)
        auction_rounds += 1
        quiet_rounds = 0 if changed else quiet_rounds + 1
        changed = False
        required_rounds = max(1, min(len(due_tasks(now)),
                                     len(AGENTS) * BUNDLE_LIMIT))
        if auction_rounds >= required_rounds * NETWORK_DIAMETER and quiet_rounds >= QUIET_ROUNDS:
            log("allocation_converged", now,
                convergence_s=now - auction_started,
                path=[TASKS[task_id][0] for task_id in path],
                reservation=reservation)
            reservation_committed = reservation is not None
            state = "IDLE"

    reserve_energy = RESERVE_FRACTION * BATTERY_CAPACITY_J
    return_energy = POWER_W * distance(position, CHARGER) / CRUISE_SPEED + reserve_energy
    if state == "TAKEOFF":
        target = (px, py, CRUISE_ALTITUDE)
        if pz >= TAKEOFF_ALTITUDE_M:
            enter_auction(now)
    elif state == "SERVICE":
        target = (px, py, CRUISE_ALTITUDE)
        if now >= service_until:
            task_deadline = deadline(active_task)
            last_completed[active_task] = now
            log("task_complete", now, task=TASKS[active_task][0],
                task_type=TASKS[active_task][1],
                deadline=task_deadline, late=now > task_deadline)
            if active_task in bundle:
                bundle.remove(active_task)
            if active_task in path:
                path.remove(active_task)
            y[active_task], z[active_task] = 0.0, None
            active_task = None
            state = "IDLE"
    elif state == "TRANSIT":
        task = TASKS[active_task]
        target = (task[2], task[3], CRUISE_ALTITUDE)
        if distance(position, task[2:4]) <= ARRIVAL_RADIUS_M:
            state = "SERVICE"
            service_until = now + task[6]
            log("task_start", now, task=task[0], duration_s=task[6])
    elif state == "IDLE":
        if path:
            task_id = path[0]
            task = TASKS[task_id]
            task_energy = POWER_W * (
                distance(position, task[2:4]) / CRUISE_SPEED + task[6] +
                distance(task[2:4], CHARGER) / CRUISE_SPEED) + reserve_energy
            if task_energy > battery:
                request_charger(now, position)
            else:
                active_task = task_id
                state = "TRANSIT"
                target = (task[2], task[3], CRUISE_ALTITUDE)
        elif reservation_committed or battery <= return_energy:
            request_charger(now, position)
        elif due_tasks(now):
            enter_auction(now)
    elif state == "TO_CHARGER":
        target = (*CHARGER, CRUISE_ALTITUDE)
        if distance(position, CHARGER) <= CHARGER_ARRIVAL_RADIUS_M:
            if mode == "proposed" and reservation and now < reservation[0]:
                state = "WAIT_RESERVATION"
            else:
                begin_charging(now)
    elif state in ("WAIT_CHARGER", "WAIT_RESERVATION"):
        angle = 2 * math.pi * me / len(AGENTS)
        target = (CHARGER[0] + HOLDING_RADIUS_M * math.cos(angle),
                  CHARGER[1] + HOLDING_RADIUS_M * math.sin(angle),
                  CRUISE_ALTITUDE)
        ready = (state == "WAIT_CHARGER" or
                 (reservation is not None and now >= reservation[0]))
        if ready and not any(peer["state"] == "CHARGING"
                             for index, peer in enumerate(peers) if index != me):
            begin_charging(now)
    elif state == "CHARGING":
        target = (*CHARGER, CRUISE_ALTITUDE)
        simultaneous = [index for index, peer in enumerate(peers)
                        if index < me and peer["state"] == "CHARGING"]
        if simultaneous:
            state = "WAIT_CHARGER"
            wait_started = now
            log("charger_conflict", now,
                occupied_by=AGENTS[min(simultaneous)])
        elif now >= charge_until:
            battery = BATTERY_CAPACITY_J
            log("charge_end", now)
            reservation = None
            enter_auction(now)
    else:  # AUCTION
        target = (px, py, CRUISE_ALTITUDE)

    delta = (target[0] - px, target[1] - py, target[2] - pz)
    remaining = math.sqrt(sum(component * component for component in delta))
    step_length = min(CRUISE_SPEED * step_seconds, remaining)
    if remaining > EPSILON:
        scale = step_length / remaining
        translation.setSFVec3f([px + scale * delta[0],
                                py + scale * delta[1],
                                pz + scale * delta[2]])
