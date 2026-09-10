"""CBBA with route-energy feasibility and shared-charger constraints."""
import json
import math
import os
from pathlib import Path

from controller import Supervisor
from mission_config import *

EPSILON = 1e-9
N_TASKS = len(TASKS)


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
if name not in AGENTS:
    # Not part of the active fleet for this run: stay parked.
    while robot.step(dt) != -1:
        pass
    raise SystemExit(0)
me = AGENTS.index(name)
project = Path(__file__).resolve().parents[2]
METHOD = os.environ.get("CR_CBBA_METHOD", "proposed").strip().lower()
if METHOD not in ("proposed", "battery_only"):
    raise ValueError("method must be proposed or battery_only")
# The ablation keeps route-energy feasibility, the hard reserve, the task
# score, CBBA task consensus, and the full-charge commitment.  It drops only
# the charger reservation: no slot is booked during bundle construction and no
# reservation is negotiated.  A UAV takes a pad if one is free when it arrives,
# and otherwise lands and waits.
RESERVE_SLOTS = METHOD == "proposed"
charger_count = int(os.environ.get(
    "CR_CBBA_CHARGER_COUNT", DEFAULT_CHARGER_COUNT))
if not 1 <= charger_count <= len(CHARGERS):
    raise ValueError("charger count must be between 1 and len(CHARGERS)")
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

output = (project / "results" / METHOD /
          f"uav{len(AGENTS)}_chargers_{charger_count}")
output.mkdir(parents=True, exist_ok=True)
log_file = (output / f"{name}.jsonl").open("w", encoding="utf-8")


def log(event, now, **fields):
    fields.update(event=event, time=round(now, 3), agent=name,
                  method=METHOD)
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
charge_complete = False
wait_started = None
target = (0.0, 0.0, CRUISE_ALTITUDE)
holding_point = CHARGER
charger_index = None
dock_index = None
dock_interval = None
standby_index = me
deferred_charge = False
takeoff_for_charger = False
ground_reservation_ready_at = None
last_status_payload = None
auction_time = 0.0
auction_battery = battery
auction_origin = None
auction_rounds = 0
quiet_rounds = 0
reservation_floor = 0.0
charger_retry_pending = False
known_feasible_unowned = set()
changed = False
messages_sent = 0
allocation_update_pending = False
charging_releases = set()
auctioned_releases = set()


def charge_duration(soc):
    """Seconds needed to reach the operational 90% target."""
    return max(0.0, TARGET_SOC - soc) / CHARGE_RATE_SOC_PER_S


def planning_battery():
    """Keep one energy snapshot throughout an allocation auction."""
    return auction_battery if state == "AUCTION" else battery


def auction_on_charger():
    return auction_origin == "CHARGER"


def controller_duration(duration):
    """Elapsed controller time for a motion leg and its state transition."""
    steps = math.ceil(max(0.0, duration - EPSILON) / step_seconds)
    return (steps + 1) * step_seconds


def travel_duration(point_a, point_b):
    return controller_duration(distance(point_a, point_b) / CRUISE_SPEED)


def descent_duration():
    return controller_duration(
        (CRUISE_ALTITUDE - PAD_ALTITUDE) / DESCENT_SPEED)


def ascent_duration():
    return controller_duration(
        (CRUISE_ALTITUDE - PAD_ALTITUDE) / ASCENT_SPEED)


def occupies_charger(agent_state):
    return agent_state in ("DESCENDING", "CHARGING", "ASCENDING")


def peer_occupies_charger(peer):
    return peer["dock_index"] is not None and (
        occupies_charger(peer["state"]) or peer["state"] == "AUCTION")


def charger_status(peer):
    return (peer_occupies_charger(peer), peer["reservation"],
            peer["reservation_committed"], peer["charger_index"],
            peer["dock_interval"], peer["dock_index"])


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
    global ground_reservation_ready_at, charger_retry_pending
    previous = reservation
    reservation = None
    reservation_committed = False
    charger_index = None
    deferred_charge = True
    takeoff_for_charger = False
    ground_reservation_ready_at = None
    charger_retry_pending = True
    if standby_index is None:
        standby_index = choose_standby(position)
    state = "TO_STANDBY"
    log("ground_charger_slot_invalidated", now, previous=previous,
        standby_index=standby_index)


def charger_position():
    index = (dock_index if dock_index is not None and state in (
        "DESCENDING", "CHARGING", "ASCENDING", "AUCTION")
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


def free_pad_now():
    """A pad no peer occupies or is currently flying to (ablation only)."""
    taken = {peer_charger_index(peer) for index, peer in enumerate(peers)
             if index != me and (peer_occupies_charger(peer) or
                                 peer["state"] == "TO_CHARGER")}
    return next((index for index in range(charger_count)
                 if index not in taken), None)


def charging_opportunity(position, now):
    """Return timing, touchdown SoC, and charger-access energy."""
    _, required, arrival, _ = route_metrics(
        position, [], now, include_reservation=False)
    if required > battery:
        return None
    route_energy = required - RESERVE_FRACTION * BATTERY_CAPACITY_J
    touchdown_soc = (battery - route_energy) / BATTERY_CAPACITY_J
    end = min(mission_duration, arrival + descent_duration() +
              charge_duration(touchdown_soc) + ascent_duration())
    access_energy = route_energy + POWER_W * ascent_duration()
    return arrival, touchdown_soc, end, access_energy


def useful_charge_time(arrival, touchdown_soc, end):
    available = end - arrival - descent_duration() - ascent_duration()
    return min(available, charge_duration(touchdown_soc))


def worthwhile_charge(opportunity, end):
    """True when replenished energy exceeds charger-access energy (III.D)."""
    arrival, touchdown_soc, _, access_energy = opportunity
    gained = (useful_charge_time(arrival, touchdown_soc, end) *
              CHARGE_RATE_SOC_PER_S * BATTERY_CAPACITY_J)
    return gained > access_energy + EPSILON


def advance_charge_slot(position, now):
    """Charging interval for a UAV between task assignments (III.D).

    With reservations, the interval is trimmed to end before the next peer
    booking on that pad, so taking it cannot displace a scheduled charge.
    Without them, a pad can only be taken if it is free at this instant.
    """
    opportunity = charging_opportunity(position, now)
    if opportunity is None:
        return None
    if not RESERVE_SLOTS:
        port = free_pad_now()
        return ((None, port) if port is not None and
                worthwhile_charge(opportunity, opportunity[2]) else None)
    arrival, touchdown_soc, full_end, _ = opportunity
    choices = []
    for port in range(charger_count):
        intervals = [interval for _, _, interval, _ in peer_intervals(port)]
        if any(other[0] <= arrival < other[1] for other in intervals):
            continue
        end = min([full_end] + [other[0] for other in intervals
                                if arrival < other[0] < full_end])
        if worthwhile_charge(opportunity, end):
            choices.append((useful_charge_time(arrival, touchdown_soc, end),
                            (arrival, end), port))
    return max(choices)[1:] if choices else None


def choose_standby(position):
    occupied = {peer["standby_index"] for index, peer in enumerate(peers)
                if index != me and peer["standby_index"] is not None and
                peer["state"] in ("TO_STANDBY", "STANDBY")}
    available = [index for index in range(len(STANDBY_POINTS))
                 if index not in occupied]
    return min(available, key=lambda index: distance(
        position, STANDBY_POINTS[index]))


def safe_charging_standby(position, altitude):
    """Find ground waiting from which later charger access remains safe."""
    occupied = {peer["standby_index"] for index, peer in enumerate(peers)
                if index != me and peer["standby_index"] is not None}
    choices = []
    for index, point in enumerate(STANDBY_POINTS):
        if index in occupied:
            continue
        landing = (travel_duration(position, point) + controller_duration(
                   max(0.0, altitude - 0.07) / DESCENT_SPEED))
        later_access = min(
            controller_duration(
                (CRUISE_ALTITUDE - 0.07) / ASCENT_SPEED) +
            travel_duration(point, dock) + descent_duration()
            for dock in CHARGERS[:charger_count])
        required = (POWER_W * (landing + later_access) +
                    RESERVE_FRACTION * BATTERY_CAPACITY_J)
        if required <= battery:
            choices.append((landing + later_access, index))
    return min(choices)[1] if choices else None


def landed_wait_site(position, altitude, now):
    """Choose a lower-energy ground route to the existing charging slot."""
    if reservation is None or charger_index is None:
        return None
    remaining = reservation[0] - now
    occupied = {peer["standby_index"] for index, peer in enumerate(peers)
                if index != me and peer["standby_index"] is not None}
    choices = []
    for index, point in enumerate(STANDBY_POINTS):
        if index in occupied:
            continue
        flight = (travel_duration(position, point) + controller_duration(
                  max(0.0, altitude - 0.07) / DESCENT_SPEED) +
                  controller_duration(
                      (CRUISE_ALTITUDE - 0.07) / ASCENT_SPEED) +
                  travel_duration(point, CHARGERS[charger_index]))
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
    endpoint, task_elapsed = route_end(position, candidate_path)
    return (required, arrival,
            required - RESERVE_FRACTION * BATTERY_CAPACITY_J,
            endpoint, task_elapsed, now)


def reservation_route_energy(profile, start, port):
    """Minimum flight energy when waiting airborne or at a ground site."""
    _, arrival, hover_energy, endpoint, task_elapsed, now = profile
    best = hover_energy + POWER_W * max(0.0, start - arrival)
    occupied = {peer["standby_index"] for index, peer in enumerate(peers)
                if index != me and peer["standby_index"] is not None}
    for index, standby in enumerate(STANDBY_POINTS):
        if index in occupied:
            continue
        flight = (task_elapsed + travel_duration(endpoint, standby) +
                  controller_duration(
                      (CRUISE_ALTITUDE - 0.07) / DESCENT_SPEED) +
                  controller_duration(
                      (CRUISE_ALTITUDE - 0.07) / ASCENT_SPEED) +
                  travel_duration(standby, CHARGERS[port]) +
                  descent_duration())
        ready = now + flight - descent_duration()
        if ready <= start + EPSILON:
            best = min(best, POWER_W * flight)
    return best


def reservation_from_profile(profile, start, port):
    if start >= mission_duration - EPSILON:
        return None
    start_soc = ((planning_battery() -
                  reservation_route_energy(profile, start, port)) /
                 BATTERY_CAPACITY_J)
    if start_soc < RESERVE_FRACTION - EPSILON:
        return None
    duration = (descent_duration() + charge_duration(start_soc) +
                ascent_duration())
    return (start, start + duration)


def reservation_from_start(position, candidate_path, now, start, port):
    """Build a variable-length dock/charge interval for a proposed start."""
    return reservation_from_profile(
        route_profile(position, candidate_path, now), start, port)


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
        elapsed += travel_duration(point, task[2:4])
        previous_start = (last_started[task_id]
                          if last_started[task_id] is not None else 0.0)
        # Urgency is evaluated at the auction instant, not at the predicted
        # arrival.  Scoring it at arrival makes a task worth more the later it
        # is scheduled, which both inverts the intended path ordering and
        # breaks the diminishing-marginal-gain condition CBBA assumes.
        revisit_age = max(1.0, (now - previous_start) / task[5])
        elapsed += math.ceil(task[6] / step_seconds) * step_seconds
        score += (task[4] * revisit_age *
                  math.exp(-LAMBDA * elapsed / 60.0))
        point = task[2:4]
    elapsed += controller_duration(
        charger_distance_bound(point) / CRUISE_SPEED)
    arrival = now + elapsed  # arrival above the pad; descent starts here
    flight_time = elapsed + descent_duration()
    arrival_energy = planning_battery() - POWER_W * flight_time
    required_energy = (POWER_W * flight_time +
                       RESERVE_FRACTION * BATTERY_CAPACITY_J)
    interval = None
    if include_reservation and arrival_energy >= (
            RESERVE_FRACTION * BATTERY_CAPACITY_J - EPSILON):
        duration = (descent_duration() +
                    charge_duration(arrival_energy / BATTERY_CAPACITY_J) +
                    ascent_duration())
        interval = (arrival, arrival + duration)
    return score, required_energy, arrival, interval


def reservation_energy(position, candidate_path, now, interval):
    """Route, waiting, and reserve energy for an assigned reservation."""
    if interval is None or charger_index is None:
        return float("inf")
    return (reservation_route_energy(
        route_profile(position, candidate_path, now), interval[0],
        charger_index) + RESERVE_FRACTION * BATTERY_CAPACITY_J)


def route_end(position, candidate_path):
    point = position
    elapsed = (ascent_duration()
               if auction_on_charger() or auction_origin == "GROUND" or
               state in ("GROUND", "STANDBY", "CHARGING") else 0.0)
    for task_id in candidate_path:
        task = TASKS[task_id]
        elapsed += (travel_duration(point, task[2:4]) +
                    math.ceil(task[6] / step_seconds) * step_seconds)
        point = task[2:4]
    return point, elapsed


def can_service_another(position, available_energy):
    reserve = RESERVE_FRACTION * BATTERY_CAPACITY_J
    return any(POWER_W * (travel_duration(position, task[2:4]) +
                          math.ceil(task[6] / step_seconds) * step_seconds +
                          controller_duration(charger_distance_bound(
                              task[2:4]) / CRUISE_SPEED) +
                          descent_duration()) +
               reserve <= available_energy
               for task in TASKS)


def needs_charge_after(position, candidate_path):
    endpoint, elapsed = route_end(position, candidate_path)
    return not can_service_another(
        endpoint, planning_battery() - POWER_W * elapsed)


def planned_reservation(position, candidate_path, now):
    if not RESERVE_SLOTS:
        return None
    if not candidate_path or not needs_charge_after(position, candidate_path):
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


def feasible_unowned_releases(position, now):
    """Released task generations that this UAV can presently accept."""
    return {(task_id, last_started[task_id]) for task_id in due_tasks(now)
            if z[task_id] in (None, me) and
            best_insertion(position, task_id, now) is not None}


def best_insertion(position, task_id, now):
    base_score = route_metrics(position, path, now)[0]
    choices = []
    for index in range(len(path) + 1):
        candidate = path[:index] + [task_id] + path[index:]
        score, energy, _, _ = route_metrics(
            position, candidate, now, include_reservation=False)
        if energy > planning_battery():
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
    choices = []
    for port in range(charger_count):
        start = interval[0]
        for _ in range(2 * len(AGENTS) + 1):
            margin = planning_battery() - (
                reservation_route_energy(profile, start, port) +
                RESERVE_FRACTION * BATTERY_CAPACITY_J)
            blockers = [interval for index, peer, interval, docked in
                        peer_intervals(port)
                        if docked or peer["reservation_committed"] or
                        (peer["margin"], index) < (margin, me)]
            candidate = reservation_from_profile(profile, start, port)
            if candidate is None:
                break
            conflicts = [other for other in blockers
                         if overlaps(candidate, other)]
            if not conflicts:
                choices.append((candidate, port))
                break
            start = min(other[1] for other in conflicts)
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
    global reservation, reservation_path, changed
    global charger_index, reservation_floor
    remote_committed = message.get(
        "reservation_committed",
        peers[message["agent"]]["reservation_committed"])
    if not RESERVE_SLOTS or reservation_committed or reservation is None:
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
    global allocation_update_pending
    released = False
    for task_id in bundle:
        if z[task_id] == me:
            y[task_id], z[task_id] = 0.0, None
            released = True
    bundle, path = [], []
    bundle_committed = False
    charge_after_bundle = False
    if clear_reservation:
        reservation = None
        charger_index = None
        reservation_path = ()
    allocation_update_pending |= released


def request_charger(now, position):
    """Execute the charger leg already committed during allocation."""
    global state, reservation, reservation_committed, holding_point
    global charger_index, standby_index, deferred_charge
    global takeoff_for_charger, charger_retry_pending
    release_bundle(clear_reservation=False)
    if reservation is None:
        if RESERVE_SLOTS:
            preferred = route_metrics(position, [], now)[3]
            result = feasible_reservation(position, [], now, preferred)
            reservation, charger_index = (result if result is not None
                                          else (None, None))
            reservation_committed = reservation is not None
            acquired = reservation is not None
        else:
            charger_index = free_pad_now()
            acquired = charger_index is not None
        if not acquired:
            standby_index = safe_charging_standby(
                position, gps.getValues()[2])
            if standby_index is None:
                fail_uav(now, "no_safe_charger_recovery")
                return
            deferred_charge = True
            charger_retry_pending = True
            takeoff_for_charger = False
            state = "TO_STANDBY"
            log("charger_deferred_to_ground", now,
                standby_index=standby_index,
                battery_soc=battery / BATTERY_CAPACITY_J)
            return
    deferred_charge = False
    charger_retry_pending = False
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


def retry_charger(now, position):
    """Recover from an unexpected occupied pad using the normal policy."""
    global reservation, reservation_committed, charger_index
    global dock_index, dock_interval
    reservation = None
    reservation_committed = False
    charger_index = None
    dock_index = None
    dock_interval = None
    request_charger(now, position)


def begin_charging(now, position):
    global state, reservation, charger_index, takeoff_for_charger
    global dock_index, dock_interval, reservation_committed
    global charge_complete
    occupied = [index for index, peer in enumerate(peers)
                if index != me and peer_occupies_charger(peer)
                and peer_charger_index(peer) == charger_index]
    if occupied:
        retry_charger(now, position)
        return
    takeoff_for_charger = False
    charge_complete = False
    dock_index = charger_index
    state = "DESCENDING"
    touchdown_soc = ((battery - POWER_W * descent_duration()) /
                     BATTERY_CAPACITY_J)
    reservation = (now, now + descent_duration() +
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
    global service_until
    global charger_index, dock_index, dock_interval, auction_origin
    if state == "FAILED":
        return
    release_bundle()
    active_task = None
    service_until = None
    reservation = None
    reservation_committed = False
    charger_index = None
    dock_index = None
    dock_interval = None
    auction_origin = None
    state = "FAILED"
    log("uav_failed", now, reason=reason,
        battery_soc=battery / BATTERY_CAPACITY_J)


log("mission_start", 0.0, initial_soc=INITIAL_SOC[me],
    charger_count=charger_count, mission_duration_s=mission_duration)
enter_auction("GROUND")
while robot.step(dt) != -1:
    now = robot.getTime()
    if now > mission_duration:
        log("mission_end", now, messages=messages_sent)
        log_file.close()
        break

    px, py, pz = gps.getValues()
    position = (px, py)
    allocation_reply = False
    charging_now = state == "CHARGING" or (
        state == "AUCTION" and auction_on_charger())
    landed_now = (state in ("GROUND", "STANDBY") or
                  (state == "AUCTION" and auction_origin == "GROUND"))
    if not charging_now and not landed_now and state != "FAILED":
        reserve_energy = RESERVE_FRACTION * BATTERY_CAPACITY_J
        next_battery = battery - POWER_W * step_seconds
        if next_battery < reserve_energy - EPSILON:
            battery = reserve_energy
            fail_uav(now, "hard_reserve_reached")
        else:
            battery = next_battery
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
            previous_charger_state = charger_status(peers[sender])
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
            current_charger_state = charger_status(peers[sender])
            if current_charger_state != previous_charger_state:
                charger_retry_pending = True
            for task_id, started_at in enumerate(message["last_started"]):
                if (started_at is not None and
                        (last_started[task_id] is None or
                         started_at > last_started[task_id])):
                    last_started[task_id] = started_at
                    if task_id != active_task and (
                            y[task_id], z[task_id]) != (0.0, None):
                        y[task_id], z[task_id] = 0.0, None
                        changed = True
        elif message["kind"] == "allocation":
            allocation_reply |= message.get("request_reply", False)
            peers[sender]["reservation"] = message.get("reservation")
            peers[sender]["reservation_committed"] = message.get(
                "reservation_committed", False)
            peers[sender]["charger_index"] = message.get("charger_index")
            peers[sender]["margin"] = message.get("margin", float("inf"))
            resolve_cbba(message, now)
            resolve_charger(
                message, auction_time if state == "AUCTION" else now, position)

    # Standard CBBA release rule (Choi, Brunet, and How, 2009): losing a task
    # invalidates the marginal scores of every task bundled after it.
    truncate_lost_bundle()

    if tuple(path) != path_before_messages and state not in (
            "TO_CHARGER", "WAIT_RESERVATION",
            "DESCENDING", "CHARGING", "ASCENDING"):
        refresh_energy_plan(position, now)

    newly_feasible_unowned = False
    if state in ("IDLE", "TO_STANDBY", "STANDBY", "CHARGING"):
        current_feasible_unowned = feasible_unowned_releases(position, now)
        newly_feasible_unowned = bool(
            current_feasible_unowned - known_feasible_unowned)
        known_feasible_unowned = current_feasible_unowned

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
            # Record every release considered by this auction so the same
            # unchanged task generation does not immediately reopen it.
            auctioned_releases.update(
                (task_id, last_started[task_id])
                for task_id in due_tasks(auction_time))
            log("allocation_converged", now,
                rounds=auction_rounds, forced=forced,
                path=[TASKS[task_id][0] for task_id in path],
                reservation=reservation)
            origin = auction_origin
            auction_origin = None
            if origin == "GROUND":
                if path:
                    takeoff_for_charger = False
                    ground_reservation_ready_at = None
                    state = "TAKEOFF"
                else:
                    state = "STANDBY"
                    # Won nothing: only queue for a charger if this UAV could
                    # not serve a task anyway.  Anything below the 90% target
                    # used to qualify, which sent UAVs to a pad with most of
                    # their endurance unused.
                    if not can_service_another(position, battery):
                        deferred_charge = True
                        if reservation is None:
                            takeoff_for_charger = False
                            ground_reservation_ready_at = None
                            charger_retry_pending = True
            elif origin == "CHARGER":
                if path:
                    dock_interval = (dock_interval[0],
                                     now + ascent_duration())
                    if not charge_complete:
                        log("charge_end", now,
                            departure_soc=battery / BATTERY_CAPACITY_J,
                            reason="task_assignment")
                        charge_complete = True
                    state = "ASCENDING"
                else:
                    state = "CHARGING"
            else:
                state = "IDLE"

    if state == "AUCTION" or allocation_reply or allocation_update_pending:
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
        allocation_update_pending = False

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
            if path or not charge_after_bundle:
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
            if task_energy > battery:
                request_charger(now, position)
            else:
                active_task = task_id
                state = "TRANSIT"
                target = (task[2], task[3], CRUISE_ALTITUDE)
        elif charge_after_bundle or not can_service_another(position, battery):
            request_charger(now, position)
        elif has_new_release(now) or newly_feasible_unowned:
            enter_auction()
        else:
            slot = advance_charge_slot(position, now)
            if slot is not None:
                reservation, charger_index = slot
                reservation_committed = reservation is not None
                log("advance_charge_planned", now, reservation=reservation,
                    battery_soc=battery / BATTERY_CAPACITY_J)
                request_charger(now, position)
            else:
                standby_index = choose_standby(position)
                charger_retry_pending = True
                state = "TO_STANDBY"
                log("standby_requested", now, standby_index=standby_index)
    elif state == "TO_CHARGER":
        dock = charger_position()
        target = (*dock, CRUISE_ALTITUDE)
        charger_distance = distance(position, dock)
        arrival = now + controller_duration(
            charger_distance / CRUISE_SPEED)
        if reservation is not None and arrival >= reservation[1]:
            missed = reservation
            reservation_committed = False
            preferred = reservation_from_start(
                position, [], now, arrival, charger_index)
            result = feasible_reservation(position, [], now, preferred)
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
            early = reservation and arrival < reservation[0]
            occupied_before_slot = (reservation and occupied and
                                    now < reservation[0])
            if (charger_distance <= HOLDING_DISTANCE_M and
                  (early or occupied_before_slot)):
                state = "WAIT_RESERVATION"
                wait_started = now
                target = (*holding_point, CRUISE_ALTITUDE)
            elif charger_distance <= HOLDING_DISTANCE_M and occupied:
                log("charger_conflict", now)
                retry_charger(now, position)
            elif charger_distance <= CHARGER_ARRIVAL_RADIUS_M:
                begin_charging(now, position)
    elif state == "WAIT_RESERVATION":
        target = (*holding_point, CRUISE_ALTITUDE)
        travel_time = travel_duration(position, charger_position())
        ready = (reservation is None or
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
            log("charger_conflict", now,
                occupied_by=AGENTS[min(blocking)])
            retry_charger(now, position)
        elif pz <= PAD_ALTITUDE + EPSILON:
            state = "CHARGING"
            log("touchdown", now,
                battery_soc=battery / BATTERY_CAPACITY_J)
            duration = charge_duration(battery / BATTERY_CAPACITY_J)
            log("charge_start", now, until=now + duration,
                duration_s=duration,
                arrival_soc=battery / BATTERY_CAPACITY_J)
    elif state == "CHARGING":
        target = (*charger_position(), PAD_ALTITUDE)
        handoff = min((interval[0] for _, _, interval, _ in
                       peer_intervals(dock_index)), default=None)
        task_release = ((has_new_release(now) and
                         has_feasible_new_release(position, now)) or
                        newly_feasible_unowned)
        if handoff is not None and now + ascent_duration() >= handoff:
            dock_interval = (dock_interval[0], handoff)
            if not charge_complete:
                log("charge_end", now,
                    departure_soc=battery / BATTERY_CAPACITY_J,
                    reason="charging_handoff")
                charge_complete = True
            state = "ASCENDING"
            target = (*charger_position(), CRUISE_ALTITUDE)
        elif charge_complete and task_release:
            # A committed charge runs to its target first.  Departing at a
            # partial state of charge collapses into a low-energy cycle of
            # one task per pad visit, which never occupies a charger long
            # enough for capacity to bind.
            enter_auction("CHARGER")
        elif (not charge_complete and battery >=
              TARGET_SOC * BATTERY_CAPACITY_J - EPSILON):
            battery = min(battery, TARGET_SOC * BATTERY_CAPACITY_J)
            log("charge_end", now,
                departure_soc=battery / BATTERY_CAPACITY_J,
                reason="charge_target")
            charge_complete = True
            dock_interval = (dock_interval[0], handoff if handoff is not None
                             else mission_duration)
    elif state == "ASCENDING":
        target = (*charger_position(), CRUISE_ALTITUDE)
        if pz >= TAKEOFF_ALTITUDE_M:
            log("dock_end", now,
                departure_soc=battery / BATTERY_CAPACITY_J)
            dock_index = None
            dock_interval = None
            if path:
                state = "IDLE"
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
        elif newly_feasible_unowned:
            # Serving a released task takes precedence over queueing for a
            # charger, including while a charger request is deferred.
            deferred_charge = False
            enter_auction()
        elif horizontal <= ARRIVAL_RADIUS_M and pz <= 0.07:
            state = "STANDBY"
            log("standby_landed", now, standby_index=standby_index)
    elif state == "STANDBY":
        target = (*STANDBY_POINTS[standby_index], 0.07)
        if newly_feasible_unowned:
            deferred_charge = False
            log("standby_departure", now, standby_index=standby_index,
                battery_soc=battery / BATTERY_CAPACITY_J)
            enter_auction("GROUND")
        elif takeoff_for_charger and not RESERVE_SLOTS:
            if now >= (ground_reservation_ready_at or now):
                standby_index = None
                state = "TAKEOFF"
        elif takeoff_for_charger:
            if charger_retry_pending:
                charger_retry_pending = False
                preferred = route_metrics(position, [], now)[3]
                earlier = feasible_reservation(
                    position, [], now, preferred)
                if (earlier is not None and
                        earlier[0][0] < reservation[0] - EPSILON):
                    reservation, charger_index = earlier
                    ground_reservation_ready_at = (
                        now + QUIET_ROUNDS * step_seconds)
                    log("charger_slot_advanced", now,
                        reservation=reservation,
                        charger_index=charger_index)
            if assigned_pad_conflict():
                replacement = feasible_reservation(
                    position, [], now, route_metrics(position, [], now)[3])
                if replacement is not None:
                    reservation, charger_index = replacement
                    ground_reservation_ready_at = (
                        now + QUIET_ROUNDS * step_seconds)
                    log("charger_slot_shifted", now,
                        reservation=reservation, charger_index=charger_index)
                else:
                    invalidate_ground_reservation(now, position)
            if takeoff_for_charger and not assigned_pad_conflict():
                travel = (ascent_duration() +
                          travel_duration(position, charger_position()))
                departure = reservation[0] - travel
                if now >= max(departure, ground_reservation_ready_at):
                    reservation_committed = True
                    standby_index = None
                    state = "TAKEOFF"
        elif charger_retry_pending:
            charger_retry_pending = False
            result = None
            if not deferred_charge:
                result = advance_charge_slot(position, now)
            elif deferred_charge:
                if RESERVE_SLOTS:
                    preferred = route_metrics(position, [], now)[3]
                    if preferred is not None:
                        preferred = (preferred[0] + ascent_duration(),
                                     preferred[1])
                    result = feasible_reservation(position, [], now, preferred)
                else:
                    port = free_pad_now()
                    result = (None, port) if port is not None else None
            if result is not None:
                reservation, charger_index = result
                reservation_committed = False
                deferred_charge = False
                takeoff_for_charger = True
                ground_reservation_ready_at = (
                    now + QUIET_ROUNDS * step_seconds)
                log("ground_charger_slot_acquired", now,
                    reservation=reservation, charger_index=charger_index)
    elif state == "FAILED":
        target = (px, py, 0.07)
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
    standby_descending = (state == "TO_STANDBY" and
                           distance(position, STANDBY_POINTS[standby_index]) <=
                           ARRIVAL_RADIUS_M)
    motion_speed = (DESCENT_SPEED if state in ("DESCENDING", "FAILED") or
                    standby_descending else
                    ASCENT_SPEED if state in ("TAKEOFF", "ASCENDING") else
                    CRUISE_SPEED)
    step_length = min(motion_speed * step_seconds, remaining)
    if remaining > EPSILON:
        scale = step_length / remaining
        translation.setSFVec3f([px + scale * delta[0],
                                py + scale * delta[1],
                                pz + scale * delta[2]])
