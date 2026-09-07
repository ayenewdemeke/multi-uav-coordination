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
charger_count = int(os.environ.get("CR_CBBA_CHARGERS", "3"))
if charger_count < 1:
    raise ValueError("CR_CBBA_CHARGERS must be a positive integer")
mission_duration = float(os.environ.get(
    "CR_CBBA_MISSION_DURATION", MISSION_DURATION_S))

gps = robot.getDevice("gps")
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
for device in (gps, receiver):
    device.enable(dt)
receiver.setChannel(1)
emitter.setChannel(1)
self_node = robot.getSelf()
translation = self_node.getField("translation")

output = project / "results" / f"chargers_{charger_count}" / mode
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
          "reservation_committed": False, "charger_index": None,
          "dock_interval": None, "dock_index": None,
          "standby_index": None, "margin": float("inf")}
         for _ in AGENTS]
reservation = None
reservation_path = ()
bundle_committed = False
reservation_committed = False
charge_after_bundle = False
state = "GROUND"
active_task = None
service_until = None
charge_until = None
charge_complete = False
connection_until = None
wait_started = None
target = (0.0, 0.0, CRUISE_ALTITUDE)
holding_point = CHARGER
charger_index = None
dock_index = None
dock_interval = None
standby_index = me
opportunistic_charge = False
deferred_charge = False
takeoff_for_charger = False
ground_reservation_ready_at = None
failure_target = None
failure_landed = False
last_status_payload = None
auction_time = 0.0
auction_battery = battery
auction_origin = None
auction_rounds = 0
quiet_rounds = 0
last_auction_end = -float("inf")
ground_retry_after = -float("inf")
reservation_floor = 0.0
changed = False
messages_sent = 0
reassignments = 0
charging_releases = set()
auctioned_releases = set()


def charge_duration(soc):
    """Seconds needed to reach the operational 90% target."""
    return max(0.0, TARGET_SOC - soc) / CHARGE_RATE_SOC_PER_S


def planning_battery():
    """Keep one energy snapshot throughout an allocation auction."""
    return auction_battery if state == "AUCTION" else battery


def admissible_energy():
    """Energy a candidate route may use, held clear of the hard reserve.

    Feasibility is evaluated against the auction-time battery snapshot, but
    the auction, the dock exit and the ascent all elapse after that estimate
    is taken, so a route planned to land exactly on the reserve can cross it
    during the final descent to the pad.
    """
    return planning_battery() - PLANNING_MARGIN_FRACTION * BATTERY_CAPACITY_J


def auction_on_charger():
    return auction_origin in ("CHARGER", "CHARGER_EXIT")


def descent_duration():
    return (CRUISE_ALTITUDE - PAD_ALTITUDE) / DESCENT_SPEED


def ascent_duration():
    return (CRUISE_ALTITUDE - PAD_ALTITUDE) / ASCENT_SPEED


def occupies_charger(agent_state):
    return agent_state in ("DESCENDING", "CONNECTING", "CHARGING",
                           "ASCENDING")


def peer_occupies_charger(peer):
    return peer["dock_index"] is not None and (
        occupies_charger(peer["state"]) or peer["state"] == "AUCTION")


def assigned_pad_conflict():
    """Detect a newly learned conflict with a ground-acquired slot."""
    if reservation is None or charger_index is None:
        return False
    return any(
        index != me and peer_charger_index(peer) == charger_index and
        ((peer_occupies_charger(peer) and
          overlaps(reservation, peer["dock_interval"])) or
         (peer["reservation_committed"] and
          overlaps(reservation, peer["reservation"]) and
          (peer["reservation"][0], index) < (reservation[0], me)))
        for index, peer in enumerate(peers))


def invalidate_ground_reservation(now, position):
    """Return to landed waiting when fresh peer state invalidates a slot."""
    global state, reservation, reservation_committed, charger_index
    global standby_index, deferred_charge, takeoff_for_charger
    global ground_reservation_ready_at, ground_retry_after
    previous = reservation
    reservation = None
    reservation_committed = False
    charger_index = None
    deferred_charge = True
    takeoff_for_charger = False
    ground_reservation_ready_at = None
    # Back off before competing again: a UAV that keeps losing the start-time
    # tie-break would otherwise spin on acquire/invalidate every step.
    ground_retry_after = now + GROUND_RETRY_BACKOFF_S
    if standby_index is None:
        standby_index = choose_standby(position)
    state = "TO_STANDBY"
    log("ground_charger_slot_invalidated", now, previous=previous,
        standby_index=standby_index)


def charger_position():
    index = (dock_index if dock_index is not None and state in (
        "DESCENDING", "CONNECTING", "CHARGING", "ASCENDING", "AUCTION")
        else charger_index)
    return CHARGERS[index] if index is not None else CHARGER


def peer_charger_index(peer):
    return peer["dock_index"] if peer_occupies_charger(peer) else peer[
        "charger_index"]


def peer_intervals(port):
    for index, peer in enumerate(peers):
        if index == me:
            continue
        if peer["reservation"] is not None and peer["charger_index"] == port:
            yield index, peer, peer["reservation"], False
        if peer["dock_interval"] is not None and peer["dock_index"] == port:
            yield index, peer, peer["dock_interval"], True


def charger_distance_bound(position):
    """Conservative distance before a capacity reservation receives a pad."""
    return max(distance(position, dock) for dock in CHARGERS[:charger_count])


def choose_charger():
    """Assign a physical pad consistently within the reserved time cohort."""
    if mode == "proposed" and reservation is not None:
        cohort = sorted([me] + [
            index for index, peer in enumerate(peers)
            if index != me and overlaps(reservation, peer["reservation"])])
        return cohort.index(me) % charger_count
    occupied = {peer_charger_index(peer)
                for index, peer in enumerate(peers)
                if index != me and (peer_occupies_charger(peer) or
                                    peer["state"] == "TO_CHARGER")}
    return next((index for index in range(charger_count)
                 if index not in occupied), 0)


def emergency_landing_point(position):
    """Move clear of known task props or the central structure before descent."""
    x, y = position
    for cx, cy, half_x, half_y in TASK_LANDING_ZONES:
        if abs(x - cx) <= half_x and abs(y - cy) <= half_y:
            edges = ((half_x - abs(x - cx),
                      cx + math.copysign(half_x + TASK_LANDING_MARGIN_M,
                                         x - cx or 1.0), y),
                     (half_y - abs(y - cy), x,
                      cy + math.copysign(half_y + TASK_LANDING_MARGIN_M,
                                         y - cy or 1.0)))
            _, x, y = min(edges)
            break
    xmin, xmax, ymin, ymax = STRUCTURE_BOUNDS
    if xmin <= x <= xmax and ymin <= y <= ymax:
        edges = ((abs(x - xmin), xmin - EMERGENCY_LANDING_MARGIN_M, y),
                 (abs(x - xmax), xmax + EMERGENCY_LANDING_MARGIN_M, y),
                 (abs(y - ymin), x, ymin - EMERGENCY_LANDING_MARGIN_M),
                 (abs(y - ymax), x, ymax + EMERGENCY_LANDING_MARGIN_M))
        _, x, y = min(edges)
    return x, y


def next_release(now):
    future = [release_time(task_id) for task_id in range(N_TASKS)
              if release_time(task_id) > now + EPSILON]
    return min(future, default=mission_duration)


def opportunistic_slot(position, now):
    """Return a free partial-charge interval ending before the next release."""
    _, required, arrival, _ = route_metrics(
        position, [], now, include_reservation=False)
    if required > battery:
        return None
    route_energy = required - RESERVE_FRACTION * BATTERY_CAPACITY_J
    touchdown_soc = (battery - route_energy) / BATTERY_CAPACITY_J
    choices = []
    for port in range(charger_count):
        end = next_release(now)
        intervals = [interval for _, _, interval, _ in peer_intervals(port)]
        conflicts = [other for other in intervals
                     if other[0] <= arrival < other[1]]
        if conflicts:
            continue
        end = min([end] + [other[0] for other in intervals
                           if arrival < other[0] < end])
        usable = end - arrival - descent_duration() - \
            DOCK_CONNECTION_S - ascent_duration()
        useful = min(usable, charge_duration(touchdown_soc))
        if useful >= MIN_OPPORTUNISTIC_CHARGE_S:
            choices.append((useful, (arrival, end), port))
    return max(choices)[1:] if choices else None


def choose_standby(position):
    occupied = {peer["standby_index"] for index, peer in enumerate(peers)
                if index != me and peer["standby_index"] is not None and
                peer["state"] in ("TO_STANDBY", "STANDBY")}
    available = [index for index in range(len(STANDBY_POINTS))
                 if index not in occupied]
    return min(available, key=lambda index: distance(
        position, STANDBY_POINTS[index]))


def landed_wait_site(position, altitude, now):
    """Choose a lower-energy ground route to the existing charging slot."""
    if mode != "proposed" or reservation is None or charger_index is None:
        return None
    remaining = reservation[0] - now
    occupied = {peer["standby_index"] for index, peer in enumerate(peers)
                if index != me and peer["standby_index"] is not None}
    choices = []
    for index, point in enumerate(STANDBY_POINTS):
        if index in occupied:
            continue
        legs = (distance(position, point) / CRUISE_SPEED,
                max(0.0, altitude - 0.07) / DESCENT_SPEED,
                (CRUISE_ALTITUDE - 0.07) / ASCENT_SPEED,
                distance(point, CHARGERS[charger_index]) / CRUISE_SPEED)
        # Round each movement leg and its state transition to controller steps.
        flight = sum(math.ceil(leg / step_seconds) * step_seconds
                     for leg in legs) + len(legs) * step_seconds
        required = (POWER_W * (flight + descent_duration()) +
                    RESERVE_FRACTION * BATTERY_CAPACITY_J)
        if flight < remaining and required <= battery:
            choices.append((flight, index))
    return min(choices)[1] if choices else None


def route_profile(position, candidate_path, now):
    """Route quantities independent of the proposed charger start time.

    The reservation searches retry a candidate start up to 2N+1 times per pad
    and the route behind it is identical every time, so it is computed once.
    """
    _, required, arrival, _ = route_metrics(
        position, candidate_path, now, include_reservation=False)
    return required, arrival, required - RESERVE_FRACTION * BATTERY_CAPACITY_J


def reservation_from_profile(profile, start):
    required, arrival, route_energy = profile
    arrival_energy = planning_battery() - route_energy
    waiting = max(0.0, start - arrival)
    start_soc = (arrival_energy - POWER_W * waiting) / BATTERY_CAPACITY_J
    if start_soc < RESERVE_FRACTION - EPSILON:
        return None
    duration = (descent_duration() + DOCK_CONNECTION_S +
                charge_duration(start_soc) + ascent_duration())
    return (start, start + duration)


def reservation_from_start(position, candidate_path, now, start):
    """Build a variable-length dock/charge interval for a proposed start."""
    return reservation_from_profile(
        route_profile(position, candidate_path, now), start)


def route_metrics(position, candidate_path, now, include_reservation=True,
                  dock_departure=False):
    point = position
    landed_departure = (auction_origin == "GROUND" or
                        state in ("GROUND", "STANDBY", "CHARGING"))
    elapsed = (ascent_duration()
               if auction_on_charger() or landed_departure or dock_departure
               else 0.0)
    score = 0.0
    for task_id in candidate_path:
        task = TASKS[task_id]
        elapsed += distance(point, task[2:4]) / CRUISE_SPEED
        previous_start = (last_started[task_id]
                          if last_started[task_id] is not None else 0.0)
        revisit_age = max(
            1.0, (now + elapsed - previous_start) / task[5])
        elapsed += task[6]
        score += (task[4] * revisit_age *
                  math.exp(-LAMBDA * elapsed / 60.0))
        point = task[2:4]
    elapsed += charger_distance_bound(point) / CRUISE_SPEED
    arrival = now + elapsed  # arrival above the pad; descent starts here
    flight_time = elapsed + descent_duration()
    arrival_energy = planning_battery() - POWER_W * flight_time
    required_energy = (POWER_W * flight_time +
                       RESERVE_FRACTION * BATTERY_CAPACITY_J)
    interval = None
    if include_reservation and arrival_energy >= (
            RESERVE_FRACTION * BATTERY_CAPACITY_J - EPSILON):
        duration = (descent_duration() + DOCK_CONNECTION_S +
                    charge_duration(arrival_energy / BATTERY_CAPACITY_J) +
                    ascent_duration())
        interval = (arrival, arrival + duration)
    return score, required_energy, arrival, interval


def reservation_energy(position, candidate_path, now, interval):
    """Route energy including any hover until an already chosen reservation."""
    required, arrival, _ = route_profile(position, candidate_path, now)
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
    reserve = (RESERVE_FRACTION + PLANNING_MARGIN_FRACTION) * \
        BATTERY_CAPACITY_J
    return any(POWER_W * (distance(position, task[2:4]) / CRUISE_SPEED +
                          task[6] + charger_distance_bound(task[2:4]) /
                          CRUISE_SPEED + descent_duration()) +
               reserve <= available_energy
               for task in TASKS)


def needs_charge_after(position, candidate_path):
    endpoint, elapsed = route_end(position, candidate_path)
    return not can_service_another(
        endpoint, planning_battery() - POWER_W * elapsed)


def planned_reservation(position, candidate_path, now):
    if mode != "proposed" or not candidate_path or not needs_charge_after(
            position, candidate_path):
        return None
    return route_metrics(position, candidate_path, now)[3]


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
    return [task_id for task_id, task in enumerate(TASKS)
            if now >= release_time(task_id) and task_id != active_task
            and task_id not in executing]


def has_new_release(now):
    return any((task_id, last_started[task_id]) not in auctioned_releases
               for task_id in due_tasks(now))


def has_feasible_new_release(position, now):
    return any(
        (task_id, last_started[task_id]) not in auctioned_releases and
        best_insertion(position, task_id, now) is not None
        for task_id in due_tasks(now))


def has_unowned_feasible_task(position, now):
    """A released task this UAV could take that no peer currently holds."""
    return any(z[task_id] in (None, me) and
               best_insertion(position, task_id, now) is not None
               for task_id in due_tasks(now))


def may_reauction(now):
    """Bound the re-auction rate so an unwinnable release cannot spin."""
    return now - last_auction_end >= AUCTION_COOLDOWN_S


def best_insertion(position, task_id, now):
    base_score = route_metrics(position, path, now)[0]
    choices = []
    for index in range(len(path) + 1):
        candidate = path[:index] + [task_id] + path[index:]
        score, energy, _, _ = route_metrics(
            position, candidate, now, include_reservation=False)
        if energy > admissible_energy():
            continue
        preferred = planned_reservation(position, candidate, now)
        if preferred is not None and feasible_reservation(
                position, candidate, now, preferred) is None:
            continue
        choices.append((score - base_score, index))
    return max(choices, default=None, key=lambda item: (item[0], -item[1]))


def build_bundle(now, position):
    """Standard CBBA phase 1: sequential greedy bundle construction."""
    global changed
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
    refresh_energy_plan(position, now)


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
    """Return the earliest feasible (interval, physical charger) pair."""
    if interval is None:
        return None
    profile = route_profile(position, candidate_path, now)
    required, arrival, _ = profile
    choices = []
    for port in range(charger_count):
        start = interval[0]
        for _ in range(2 * len(AGENTS) + 1):
            margin = planning_battery() - (
                required + POWER_W * max(0.0, start - arrival))
            blockers = [interval for index, peer, interval, docked in
                        peer_intervals(port)
                        if docked or peer["reservation_committed"] or
                        (peer["margin"], index) < (margin, me)]
            candidate = reservation_from_profile(profile, start)
            if candidate is None:
                break
            conflicts = [other for other in blockers
                         if overlaps(candidate, other)]
            if not conflicts:
                choices.append((candidate, port))
                break
            start = min(other[1] for other in conflicts)
    return min(choices, default=None, key=lambda choice: choice[0])


def next_open_reservation(position, candidate_path, now, start):
    """Find an open (interval, physical charger) pair against all slots."""
    profile = route_profile(position, candidate_path, now)
    choices = []
    for port in range(charger_count):
        port_start = start
        intervals = [interval for _, _, interval, _ in peer_intervals(port)]
        for _ in range(2 * len(AGENTS) + 1):
            candidate = reservation_from_profile(profile, port_start)
            if candidate is None:
                break
            conflicts = [other for other in intervals
                         if overlaps(candidate, other)]
            if not conflicts:
                choices.append((candidate, port))
                break
            port_start = min(other[1] for other in conflicts)
    return min(choices, default=None, key=lambda choice: choice[0])


def refresh_energy_plan(position, now):
    """Keep charging need and reservation derived from the current route."""
    global reservation, reservation_path, reservation_committed
    global charge_after_bundle, changed, charger_index, reservation_floor
    new_charge_after = bool(path) and needs_charge_after(position, path)
    preferred = (planned_reservation(position, path, now)
                 if new_charge_after else None)
    result = (feasible_reservation(position, path, now, preferred)
              if preferred is not None else None)
    updated, updated_port = result if result is not None else (None, None)
    route_changed = tuple(path) != reservation_path
    plan_changed = (updated != reservation or updated_port != charger_index or
                    new_charge_after != charge_after_bundle)
    reservation_path = tuple(path)
    charge_after_bundle = new_charge_after
    if route_changed:
        reservation_floor = 0.0   # a different route admits a different slot
    if route_changed or plan_changed:
        reservation = updated
        charger_index = updated_port
        reservation_committed = False
        changed = True


def resolve_charger(message, now, position):
    global reservation, reservation_path, reassignments, changed
    global charger_index, reservation_floor
    remote = message.get("reservation")
    remote_committed = message.get(
        "reservation_committed",
        peers[message["agent"]]["reservation_committed"])
    if mode != "proposed" or reservation_committed or reservation is None:
        return
    my_margin = planning_battery() - reservation_energy(
        position, path, now, reservation)
    remote_margin = message.get("margin", float("inf"))
    sender = message["agent"]
    result = feasible_reservation(position, path, now, reservation)
    if result == (reservation, charger_index):
        return
    if result is not None and result[0][0] < reservation_floor - EPSILON:
        # A slot yielded within an auction is never reclaimed earlier in that
        # same auction.  The reservation start is therefore monotone, which
        # terminates the shift/counter-shift cycle between peers that
        # otherwise prevents the auction from ever reaching QUIET_ROUNDS.
        return
    if remote_committed or (my_margin, me) > (remote_margin, sender):
        released_tasks = []
        if result is not None:
            candidate, candidate_port = result
            if result != (reservation, charger_index):
                reservation = candidate
                charger_index = candidate_port
                reservation_path = tuple(path)
                reservation_floor = max(reservation_floor, candidate[0])
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
            preferred = planned_reservation(position, path, now)
            result = feasible_reservation(position, path, now, preferred)
            if result is not None or not needs_charge_after(position, path):
                reservation, charger_index = (result if result is not None
                                               else (None, None))
                reservation_path = tuple(path)
                break
        if released_tasks:
            if not bundle:
                reservation = None
                charger_index = None
                reservation_path = ()
            reassignments += len(released_tasks)
            changed = True
            log("charger_conflict_resolved", now,
                with_agent=AGENTS[sender],
                released_tasks=released_tasks,
                released_count=len(released_tasks),
                reservation=reservation)


def enter_auction(origin="AIR"):
    global state, auction_time, auction_rounds, quiet_rounds, changed
    global auction_battery, auction_origin, reservation_floor
    global bundle_committed, reservation_committed, charge_after_bundle
    state = "AUCTION"
    reservation_floor = 0.0
    auction_origin = origin
    auction_time = robot.getTime()
    auction_battery = battery
    auction_rounds = 0
    quiet_rounds = 0
    changed = True
    bundle_committed = False
    reservation_committed = False
    charge_after_bundle = False
    charging_releases.clear()


def release_bundle(clear_reservation=True):
    global bundle, path, reservation, reservation_path
    global bundle_committed, charge_after_bundle, charger_index
    for task_id in bundle:
        if z[task_id] == me:
            y[task_id], z[task_id] = 0.0, None
    bundle, path = [], []
    bundle_committed = False
    charge_after_bundle = False
    if clear_reservation:
        reservation = None
        charger_index = None
        reservation_path = ()


def request_charger(now, position):
    """Execute the charger leg already committed during allocation."""
    global state, reservation, reservation_committed, holding_point
    global charger_index, standby_index, deferred_charge
    global takeoff_for_charger
    release_bundle(clear_reservation=False)
    if reservation is None and mode == "proposed":
        preferred = route_metrics(position, [], now)[3]
        result = feasible_reservation(position, [], now, preferred)
        if result is None and preferred is not None:
            result = next_open_reservation(
                position, [], now, preferred[0])
        reservation, charger_index = (result if result is not None
                                      else (None, None))
        reservation_committed = reservation is not None
        if reservation is None:
            deferred_charge = True
            takeoff_for_charger = False
            if standby_index is None:
                standby_index = choose_standby(position)
            state = "TO_STANDBY"
            log("charger_deferred_to_ground", now,
                standby_index=standby_index,
                battery_soc=battery / BATTERY_CAPACITY_J)
            return
    deferred_charge = False
    if charger_index is None:
        charger_index = choose_charger()
    dock = charger_position()
    approach = distance(position, dock)
    if approach > EPSILON:
        scale = HOLDING_DISTANCE_M / approach
        holding_point = (dock[0] + (position[0] - dock[0]) * scale,
                         dock[1] + (position[1] - dock[1]) * scale)
    state = "TO_CHARGER"
    log("charger_request", now, reservation=reservation,
        charger_index=charger_index,
        battery_soc=battery / BATTERY_CAPACITY_J)


def begin_charging(now):
    global state, wait_started, reservation, charger_index
    global takeoff_for_charger
    global dock_index, dock_interval, reservation_committed
    global charge_complete
    occupied = [index for index, peer in enumerate(peers)
                if index != me and peer_occupies_charger(peer)
                and peer_charger_index(peer) == charger_index]
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
    if charger_index is None:
        charger_index = choose_charger()
    takeoff_for_charger = False
    charge_complete = False
    dock_index = charger_index
    state = "DESCENDING"
    if not opportunistic_charge:
        touchdown_soc = ((battery - POWER_W * descent_duration()) /
                         BATTERY_CAPACITY_J)
        reservation = (now, now + descent_duration() + DOCK_CONNECTION_S +
                       charge_duration(touchdown_soc) + ascent_duration())
    dock_interval = reservation
    reservation = None
    reservation_committed = False
    charger_index = None
    log("dock_start", now, reservation=dock_interval,
        charger_index=dock_index,
        battery_soc=battery / BATTERY_CAPACITY_J)


def fail_uav(now, reason):
    """Safely remove a UAV that cannot preserve the hard SoC reserve."""
    global state, reservation, reservation_committed, active_task
    global service_until, charge_until, connection_until
    global failure_target, failure_landed, opportunistic_charge
    global charger_index, dock_index, dock_interval, auction_origin
    if state == "FAILED":
        return
    release_bundle()
    active_task = None
    service_until = None
    charge_until = None
    connection_until = None
    reservation = None
    reservation_committed = False
    charger_index = None
    dock_index = None
    dock_interval = None
    auction_origin = None
    opportunistic_charge = False
    failure_target = emergency_landing_point((gps.getValues()[0],
                                              gps.getValues()[1]))
    failure_landed = False
    state = "FAILED"
    log("uav_failed", now, reason=reason,
        battery_soc=battery / BATTERY_CAPACITY_J)


log("mission_start", 0.0, initial_soc=INITIAL_SOC[me],
    charger_count=charger_count, mission_duration_s=mission_duration)
enter_auction("GROUND")
while robot.step(dt) != -1:
    now = robot.getTime()
    if now > mission_duration:
        log("mission_end", now, minimum_soc=minimum_soc, messages=messages_sent,
            charging_reassignments=reassignments)
        log_file.close()
        break

    px, py, pz = gps.getValues()
    position = (px, py)
    allocation_reply = False
    charging_now = state == "CHARGING" or (
        state == "AUCTION" and auction_on_charger())
    landed_now = (state in ("GROUND", "STANDBY") or
                  (state == "AUCTION" and auction_origin == "GROUND"))
    if state != "CONNECTING" and not charging_now and not landed_now and state != "FAILED":
        reserve_energy = RESERVE_FRACTION * BATTERY_CAPACITY_J
        next_battery = battery - POWER_W * step_seconds
        if next_battery < reserve_energy - EPSILON:
            battery = reserve_energy
            fail_uav(now, "hard_reserve_reached")
        else:
            battery = next_battery
        minimum_soc = min(minimum_soc, battery / BATTERY_CAPACITY_J)
    elif charging_now:
        battery = min(TARGET_SOC * BATTERY_CAPACITY_J,
                      battery + CHARGE_RATE_SOC_PER_S *
                      BATTERY_CAPACITY_J * step_seconds)

    path_before_messages = tuple(path)
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
                                     "reservation_committed", False),
                                 charger_index=message.get("charger_index"),
                                 dock_interval=message.get("dock_interval"),
                                 dock_index=message.get("dock_index"),
                                 standby_index=message.get("standby_index"))
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
            peers[sender]["charger_index"] = message.get("charger_index")
            peers[sender]["margin"] = message.get("margin", float("inf"))
            resolve_cbba(message, now)
            if state == "AUCTION":
                resolve_charger(message, auction_time, position)

    if tuple(path) != path_before_messages and state not in (
            "TO_CHARGER", "WAIT_CHARGER", "WAIT_RESERVATION",
            "DESCENDING", "CONNECTING", "CHARGING", "ASCENDING"):
        refresh_energy_plan(position, now)

    if state == "AUCTION":
        truncate_lost_bundle()
        build_bundle(auction_time, position)
        auction_rounds += 1
        quiet_rounds = 0 if changed else quiet_rounds + 1
        changed = False
        forced = auction_rounds >= AUCTION_ROUND_LIMIT
        if quiet_rounds >= QUIET_ROUNDS or forced:
            bundle_committed = bool(bundle)
            charge_after_bundle = needs_charge_after(position, path)
            reservation_committed = reservation is not None
            # Record every release this auction considered, not only those
            # won or already owned.  Re-running an identical auction on an
            # unwinnable release cannot change its outcome; a task nobody
            # claimed is still picked up through has_unowned_feasible_task.
            auctioned_releases.update(
                (task_id, last_started[task_id])
                for task_id in due_tasks(auction_time))
            last_auction_end = now
            log("allocation_converged", now,
                rounds=auction_rounds, forced=forced,
                path=[TASKS[task_id][0] for task_id in path],
                reservation=reservation)
            origin = auction_origin
            auction_origin = None
            if origin == "GROUND":
                if path:
                    state = "TAKEOFF"
                else:
                    state = "STANDBY"
                    # Won nothing: resume competing for a charger slot rather
                    # than waiting on the ground with the request cleared.
                    if battery < TARGET_SOC * BATTERY_CAPACITY_J - EPSILON:
                        deferred_charge = True
            elif origin in ("CHARGER", "CHARGER_EXIT"):
                departing = bool(path) or origin == "CHARGER_EXIT"
                if departing:
                    dock_interval = (dock_interval[0],
                                     now + ascent_duration())
                    if not charge_complete:
                        log("charge_end", now,
                            departure_soc=battery / BATTERY_CAPACITY_J,
                            reason=("task_assignment" if path else
                                    "opportunistic_window_end"))
                        charge_complete = True
                    state = "ASCENDING"
                else:
                    state = "CHARGING"
            else:
                state = "IDLE"

    if state == "AUCTION" or allocation_reply:
        margin = (planning_battery() - reservation_energy(
                      position, path, auction_time, reservation) if reservation
                  else planning_battery() -
                  RESERVE_FRACTION * BATTERY_CAPACITY_J)
        s[me] = now
        transmit("allocation", now, y=y, z=z, s=s,
                 reservation=reservation, margin=margin,
                 charger_index=charger_index,
                 reservation_committed=reservation_committed,
                 request_reply=state == "AUCTION")

    if (state in ("TAKEOFF", "TO_CHARGER", "TO_STANDBY") and takeoff_for_charger and
            assigned_pad_conflict()):
        invalidate_ground_reservation(now, position)

    if state in ("TO_CHARGER", "WAIT_RESERVATION") and not takeoff_for_charger:
        landing = landed_wait_site(position, pz, now)
        if landing is not None:
            standby_index = landing
            takeoff_for_charger = True
            deferred_charge = False
            ground_reservation_ready_at = now
            if wait_started is not None:
                log("charger_wait_end", now, duration_s=now - wait_started)
                wait_started = None
            state = "TO_STANDBY"
            log("charger_wait_landing", now, standby_index=landing,
                reservation=reservation, charger_index=charger_index)

    if state == "TAKEOFF":
        target = (px, py, CRUISE_ALTITUDE)
        if pz >= TAKEOFF_ALTITUDE_M:
            if takeoff_for_charger:
                request_charger(now, position)
            elif path:
                standby_index = None
                state = "IDLE"
            else:
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
            refresh_energy_plan(position, now)
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
            if task_energy > battery - PLANNING_MARGIN_FRACTION * \
                    BATTERY_CAPACITY_J:
                request_charger(now, position)
            else:
                active_task = task_id
                state = "TRANSIT"
                target = (task[2], task[3], CRUISE_ALTITUDE)
        elif charge_after_bundle or not can_service_another(position, battery):
            request_charger(now, position)
        elif (has_new_release(now) or
              has_unowned_feasible_task(position, now)):
            if may_reauction(now):
                enter_auction()
            else:
                target = (px, py, CRUISE_ALTITUDE)   # hold out the cooldown
        else:
            slot = opportunistic_slot(position, now)
            if slot is not None:
                reservation, charger_index = slot
                reservation_committed = True
                opportunistic_charge = True
                log("opportunistic_charge_planned", now,
                    reservation=reservation)
                request_charger(now, position)
            else:
                standby_index = choose_standby(position)
                state = "TO_STANDBY"
                log("standby_requested", now,
                    standby_index=standby_index)
    elif state == "TO_CHARGER":
        dock = charger_position()
        target = (*dock, CRUISE_ALTITUDE)
        charger_distance = distance(position, dock)
        arrival = now + charger_distance / CRUISE_SPEED
        if (mode == "proposed" and reservation is not None and
                arrival >= reservation[1]):
            missed = reservation
            reservation_committed = False
            preferred = reservation_from_start(position, [], now, arrival)
            result = feasible_reservation(position, [], now, preferred)
            if result is None:
                result = next_open_reservation(
                    position, [], now, arrival)
            reservation, charger_index = (result if result is not None
                                          else (None, None))
            reservation_committed = reservation is not None
            log("reservation_missed", now, previous=missed,
                reservation=reservation)
            if reservation is None:
                fail_uav(now, "missed_reservation_no_feasible_recovery")
        if state != "FAILED":
            occupied = any(peer_occupies_charger(peer)
                           and peer_charger_index(peer) == charger_index
                           for index, peer in enumerate(peers) if index != me)
            early = (mode == "proposed" and reservation and
                     now + charger_distance / CRUISE_SPEED < reservation[0])
            if charger_distance <= HOLDING_DISTANCE_M and (occupied or early):
                state = ("WAIT_RESERVATION" if mode == "proposed" else
                         "WAIT_CHARGER")
                wait_started = now
                if occupied and mode == "baseline":
                    log("charger_conflict", now)
                target = (*holding_point, CRUISE_ALTITUDE)
            elif charger_distance <= CHARGER_ARRIVAL_RADIUS_M:
                begin_charging(now)
    elif state in ("WAIT_CHARGER", "WAIT_RESERVATION"):
        target = (*holding_point, CRUISE_ALTITUDE)
        travel_time = distance(position, charger_position()) / CRUISE_SPEED
        ready = (state == "WAIT_CHARGER" or reservation is None or
                 now + travel_time >= reservation[0])
        occupied = any(peer_occupies_charger(peer)
                       and peer_charger_index(peer) == charger_index
                       for index, peer in enumerate(peers) if index != me)
        if ready and not occupied:
            if wait_started is not None:
                log("charger_wait_end", now, duration_s=now - wait_started)
                wait_started = None
            state = "TO_CHARGER"
            target = (*charger_position(), CRUISE_ALTITUDE)
    elif state == "DESCENDING":
        target = (*charger_position(), PAD_ALTITUDE)
        blocking = [index for index, peer in enumerate(peers)
                    if index != me and peer_charger_index(peer) == dock_index and
                    (peer_occupies_charger(peer) and
                     (peer["state"] != "DESCENDING" or index < me))]
        if blocking:
            state = "WAIT_RESERVATION" if mode == "proposed" else "WAIT_CHARGER"
            wait_started = now
            log("charger_conflict", now,
                occupied_by=AGENTS[min(blocking)])
        elif pz <= PAD_ALTITUDE + EPSILON:
            state = "CONNECTING"
            connection_until = now + DOCK_CONNECTION_S
            log("touchdown", now, connection_until=connection_until,
                battery_soc=battery / BATTERY_CAPACITY_J)
    elif state == "CONNECTING":
        target = (*charger_position(), PAD_ALTITUDE)
        if now >= connection_until:
            state = "CHARGING"
            duration = charge_duration(battery / BATTERY_CAPACITY_J)
            if opportunistic_charge:
                duration = min(duration, max(
                    0.0, dock_interval[1] - now - ascent_duration()))
            charge_until = now + duration
            log("charge_start", now, until=charge_until,
                duration_s=duration,
                arrival_soc=battery / BATTERY_CAPACITY_J)
    elif state == "CHARGING":
        target = (*charger_position(), PAD_ALTITUDE)
        task_release = (has_new_release(now) and
                        has_feasible_new_release(position, now))
        if task_release:
            enter_auction("CHARGER")
        elif opportunistic_charge and now >= charge_until:
            enter_auction("CHARGER_EXIT")
        elif (not charge_complete and battery >=
              TARGET_SOC * BATTERY_CAPACITY_J - EPSILON):
            battery = min(battery, TARGET_SOC * BATTERY_CAPACITY_J)
            log("charge_end", now,
                departure_soc=battery / BATTERY_CAPACITY_J,
                reason="charge_target")
            charge_complete = True
            dock_interval = (dock_interval[0], mission_duration)
    elif state == "ASCENDING":
        target = (*charger_position(), CRUISE_ALTITUDE)
        if pz >= TAKEOFF_ALTITUDE_M:
            log("dock_end", now,
                departure_soc=battery / BATTERY_CAPACITY_J)
            was_opportunistic = opportunistic_charge
            dock_index = None
            dock_interval = None
            opportunistic_charge = False
            if path:
                state = "IDLE"
            elif was_opportunistic and not has_new_release(now):
                reservation = None
                reservation_committed = False
                charger_index = None
                standby_index = choose_standby(position)
                state = "TO_STANDBY"
            else:
                reservation = None
                reservation_committed = False
                charger_index = None
                enter_auction()
    elif state == "TO_STANDBY":
        standby = STANDBY_POINTS[standby_index]
        horizontal = distance(position, standby)
        target = ((*standby, pz) if horizontal > ARRIVAL_RADIUS_M else
                  (*standby, 0.07))
        landing_conflict = takeoff_for_charger and any(
            index != me and peer["standby_index"] == standby_index and
            (peer["state"] != "TO_STANDBY" or index < me)
            for index, peer in enumerate(peers))
        if landing_conflict:
            standby_index = None
            takeoff_for_charger = False
            request_charger(now, position)
            target = (px, py, pz)
        elif (not takeoff_for_charger and may_reauction(now) and
              has_unowned_feasible_task(position, now)):
            # Serving a released task takes precedence over queueing for a
            # charger, including while a charger request is deferred.
            deferred_charge = False
            enter_auction()
        elif horizontal <= ARRIVAL_RADIUS_M and pz <= 0.07:
            state = "STANDBY"
            log("standby_landed", now, standby_index=standby_index)
    elif state == "STANDBY":
        target = (*STANDBY_POINTS[standby_index], 0.07)
        if takeoff_for_charger:
            if assigned_pad_conflict():
                invalidate_ground_reservation(now, position)
            else:
                travel = (ascent_duration() + distance(
                    position, charger_position()) / CRUISE_SPEED)
                departure = reservation[0] - travel
                if now >= max(departure, ground_reservation_ready_at):
                    reservation_committed = True
                    standby_index = None
                    state = "TAKEOFF"
        elif may_reauction(now) and has_unowned_feasible_task(position, now):
            # Ordered ahead of the charger branch deliberately.  A landed UAV
            # with the energy to serve a released task must be able to return
            # to the auction: deferred_charge is re-armed by every slot
            # invalidation and previously shadowed this branch permanently,
            # stranding a healthy UAV on the ground for the rest of the
            # mission while its tasks went late.
            deferred_charge = False
            log("standby_departure", now, standby_index=standby_index,
                battery_soc=battery / BATTERY_CAPACITY_J)
            enter_auction("GROUND")
        elif deferred_charge and now >= ground_retry_after:
            preferred = route_metrics(position, [], now)[3]
            if preferred is not None:
                preferred = (preferred[0] + ascent_duration(), preferred[1])
            result = feasible_reservation(position, [], now, preferred)
            if result is not None:
                reservation, charger_index = result
                reservation_committed = True
                deferred_charge = False
                takeoff_for_charger = True
                ground_reservation_ready_at = (
                    now + QUIET_ROUNDS * step_seconds)
                log("ground_charger_slot_acquired", now,
                    reservation=reservation, charger_index=charger_index)
            else:
                ground_retry_after = now + GROUND_RETRY_BACKOFF_S
    elif state == "FAILED":
        horizontal = distance(position, failure_target)
        target = ((*failure_target, pz) if horizontal > ARRIVAL_RADIUS_M else
                  (*failure_target, 0.07))
        if not failure_landed and horizontal <= ARRIVAL_RADIUS_M and pz <= 0.07:
            failure_landed = True
            log("uav_landed_out_of_service", now, position=failure_target)
    else:  # AUCTION
        if auction_on_charger():
            target = (*charger_position(), PAD_ALTITUDE)
        elif auction_origin == "GROUND":
            target = (px, py, pz)
        else:
            target = (px, py, CRUISE_ALTITUDE)

    status_payload = dict(state=state, active=active_task, bundle=bundle,
                          reservation=reservation,
                          bundle_committed=bundle_committed,
                          reservation_committed=reservation_committed,
                          charger_index=charger_index,
                          dock_interval=dock_interval,
                          dock_index=dock_index,
                          standby_index=standby_index,
                          last_started=last_started)
    status_signature = json.dumps(status_payload, sort_keys=True)
    if status_signature != last_status_payload:
        last_status_payload = status_signature
        transmit("status", now, **status_payload)

    delta = (target[0] - px, target[1] - py, target[2] - pz)
    remaining = math.sqrt(sum(component * component for component in delta))
    failure_descending = (state == "FAILED" and
                          distance(position, failure_target) <=
                          ARRIVAL_RADIUS_M)
    standby_descending = (state == "TO_STANDBY" and
                           distance(position, STANDBY_POINTS[standby_index]) <=
                           ARRIVAL_RADIUS_M)
    motion_speed = (DESCENT_SPEED if state == "DESCENDING" or
                    failure_descending or standby_descending else
                    ASCENT_SPEED if state in ("TAKEOFF", "ASCENDING") else
                    CRUISE_SPEED)
    step_length = min(motion_speed * step_seconds, remaining)
    if remaining > EPSILON:
        scale = step_length / remaining
        translation.setSFVec3f([px + scale * delta[0],
                                py + scale * delta[1],
                                pz + scale * delta[2]])
