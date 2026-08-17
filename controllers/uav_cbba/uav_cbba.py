import json
import math
from controller import Robot

TASKS = [('S1_site_entrance', -8, -18), ('S2_slab_edge_east', 6.2, 0),
         ('S3_laydown_yard', -15, 10), ('Q1_wall_quality', -4, -4),
         ('S4_fence_north', 0, 20), ('S5_fence_east', 22, 0),
         ('S6_fence_west', -22, -4), ('Q2_material', 12, 10)]
AGENTS = ['UAV1', 'UAV2', 'UAV3', 'UAV4']
N, L = len(TASKS), 2
NETWORK_DIAMETER = 1  # All UAVs broadcast directly to every other UAV.
N_MIN = min(N, len(AGENTS) * L)
CONVERGENCE_ROUNDS = N_MIN * NETWORK_DIAMETER
QUIET_ROUNDS = NETWORK_DIAMETER + 1
BID_EPSILON = 1e-9


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


def wins(b1, a1, b2, a2):
    return b1 > b2 + BID_EPSILON or (abs(b1 - b2) <= BID_EPSILON and
                                     a1 is not None and
                                     (a2 is None or a1 < a2))


robot = Robot()
dt = int(robot.getBasicTimeStep())
name = robot.getName()
me = AGENTS.index(name)
imu = robot.getDevice('inertial unit')
gps = robot.getDevice('gps')
gyro = robot.getDevice('gyro')
receiver = robot.getDevice('receiver')
emitter = robot.getDevice('emitter')
for sensor in (imu, gps, gyro, receiver):
    sensor.enable(dt)
receiver.setChannel(1)
emitter.setChannel(1)
motors = [robot.getDevice(n) for n in ('front left propeller',
          'front right propeller', 'rear left propeller',
          'rear right propeller')]
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(1)

start = None
bundle, path = [], []
y, z, s = [0.0] * N, [None] * N, [0.0] * len(AGENTS)
state, next_round, auction_rounds, stable, changed = 'AUCTION', 0, 0, 0, False
route_index, target = 0, (0, 0, 12)


def value(route):
    current_x, current_y, distance, total = start[0], start[1], 0, 0
    for j in route:
        tx, ty = TASKS[j][1:]
        distance += math.hypot(tx - current_x, ty - current_y)
        total += 100 * 0.95 ** (distance / 5)
        current_x, current_y = tx, ty
    return total


def insertion(j):
    base = value(path)
    choices = [(value(path[:n] + [j] + path[n:]) - base, n)
               for n in range(len(path) + 1)]
    return max(choices, key=lambda q: (q[0], -q[1]))


def resolve(msg, received_at):
    global changed
    k, yk, zk, sk = msg['agent'], msg['y'], msg['z'], msg['s']
    for j in range(N):
        a, b, action = zk[j], z[j], 'leave'
        if a == k:
            if b == me: action = 'update' if wins(yk[j], k, y[j], me) else 'leave'
            elif b == k or b is None: action = 'update'
            else: action = 'update' if sk[b] > s[b] or wins(yk[j], k, y[j], b) else 'leave'
        elif a == me:
            if b == k: action = 'reset'
            elif b not in (me, None) and sk[b] > s[b]: action = 'reset'
        elif a is not None:
            if b == me: action = 'update' if sk[a] > s[a] and wins(yk[j], a, y[j], me) else 'leave'
            elif b == k: action = 'update' if sk[a] > s[a] else 'reset'
            elif b == a: action = 'update' if sk[a] > s[a] else 'leave'
            elif b is None: action = 'update' if sk[a] > s[a] else 'leave'
            else:
                if sk[a] > s[a] and (sk[b] > s[b] or wins(yk[j], a, y[j], b)): action = 'update'
                elif sk[b] > s[b] and s[a] > sk[a]: action = 'reset'
        elif b == k or (b not in (me, None) and sk[b] > s[b]):
            action = 'update'
        if action == 'update' and (y[j], z[j]) != (yk[j], a):
            y[j], z[j], changed = yk[j], a, True
        elif action == 'reset' and (y[j], z[j]) != (0.0, None):
            y[j], z[j], changed = 0.0, None, True
    for m in range(len(AGENTS)):
        if m not in (me, k): s[m] = max(s[m], sk[m])
    s[k] = received_at


while robot.step(dt) != -1:
    now = robot.getTime()
    roll, pitch, yaw = imu.getRollPitchYaw()
    px, py, pz = gps.getValues()
    roll_rate, pitch_rate, _ = gyro.getValues()
    if start is None: start = (px, py)

    while receiver.getQueueLength():
        msg = json.loads(bytes(receiver.getBytes()).decode())
        receiver.nextPacket()
        if state == 'AUCTION' and msg['agent'] != me:
            resolve(msg, now)

    if state == 'AUCTION':
        lost = next((n for n, j in enumerate(bundle) if z[j] != me), None)
        if lost is not None:
            for j in bundle[lost + 1:]: y[j], z[j] = 0.0, None
            bundle = bundle[:lost]
            path = [j for j in path if j in bundle]
            changed = True

    if state == 'AUCTION' and now >= next_round:
        next_round = now + 0.5
        while len(bundle) < L:
            bids = [(insertion(j)[0], j, insertion(j)[1]) for j in range(N)
                    if j not in bundle]
            bids = [q for q in bids if wins(q[0], me, y[q[1]], z[q[1]])]
            if not bids: break
            bid, j, n = max(bids, key=lambda q: (q[0], -q[1]))
            bundle.append(j)
            path.insert(n, j)
            y[j], z[j], changed = bid, me, True
        s[me] = now
        emitter.send(json.dumps({'agent': me, 'y': y, 'z': z,
                                 's': s}).encode())
        auction_rounds += 1
        stable = 0 if changed else stable + 1
        changed = False
        if (auction_rounds >= CONVERGENCE_ROUNDS and
                stable >= QUIET_ROUNDS and len(bundle) == L):
            print(f'{name} -> ' + ' -> '.join(TASKS[j][0] for j in path), flush=True)
            state = 'TAKEOFF'

    if state == 'AUCTION': continue
    if state == 'TAKEOFF':
        target = (px, py, 12)
        if pz > 11.5: state = 'TRANSIT'
    else:
        j = path[route_index]
        target = (TASKS[j][1], TASKS[j][2], 12)
        if math.hypot(target[0] - px, target[1] - py) < 1:
            if route_index + 1 < len(path): route_index += 1
            else: state = 'HOLD'

    dx, dy = target[0] - px, target[1] - py
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    left = -math.sin(yaw) * dx + math.cos(yaw) * dy
    pitch_move = clamp(-0.4 * forward, -2, 2)
    roll_move = clamp(0.4 * left, -2, 2)
    yaw_move = 0
    roll_input = 50 * clamp(roll, -1, 1) + roll_rate + roll_move
    pitch_input = 30 * clamp(pitch, -1, 1) + pitch_rate + pitch_move
    vertical = 3 * clamp(target[2] - pz + 0.6, -1, 1) ** 3
    base = 68.5 + vertical
    motors[0].setVelocity(base - roll_input + pitch_input - yaw_move)
    motors[1].setVelocity(-(base + roll_input + pitch_input + yaw_move))
    motors[2].setVelocity(-(base - roll_input - pitch_input + yaw_move))
    motors[3].setVelocity(base + roll_input - pitch_input - yaw_move)
