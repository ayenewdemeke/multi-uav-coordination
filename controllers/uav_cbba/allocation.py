"""Pure planning helpers for energy- and charging-constrained CBBA."""

import math

from mission_config import (BATTERY_CAPACITY_J, CHARGER, CHARGER_ACCESS_S,
                            CRUISE_SPEED, LAMBDA, POWER_W, RESERVE_FRACTION,
                            TASKS)


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def overlaps(a, b):
    return a is not None and b is not None and a[0] < b[1] and b[0] < a[1]


def route_metrics(position, path, now):
    """Return score, energy, finish time and predicted charger interval."""
    point = position
    elapsed = 0.0
    score = 0.0
    for task_id in path:
        task = TASKS[task_id]
        elapsed += distance(point, task[2:4]) / CRUISE_SPEED + task[6]
        score += task[4] * math.exp(-LAMBDA * elapsed)
        point = task[2:4]
    elapsed += distance(point, CHARGER) / CRUISE_SPEED
    energy = POWER_W * elapsed + RESERVE_FRACTION * BATTERY_CAPACITY_J
    arrival = now + elapsed
    return score, energy, arrival, (arrival, arrival + CHARGER_ACCESS_S)


def best_insertion(position, path, task_id, now, battery_j, reservations,
                   owner, charger_aware):
    base = route_metrics(position, path, now)[0]
    choices = []
    for index in range(len(path) + 1):
        candidate = path[:index] + [task_id] + path[index:]
        score, energy, arrival, reservation = route_metrics(position, candidate, now)
        if energy > battery_j:
            continue
        if charger_aware and any(agent != owner and overlaps(reservation, interval)
                                 for agent, interval in reservations.items()):
            continue
        choices.append((score - base, index, reservation, energy))
    return max(choices, default=None, key=lambda item: (item[0], -item[1]))


def reservation_winner(agent_a, margin_a, agent_b, margin_b):
    """Smaller energy margin wins; agent name deterministically breaks ties."""
    return agent_a if (margin_a, agent_a) < (margin_b, agent_b) else agent_b

