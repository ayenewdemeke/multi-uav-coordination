# CR-CBBA

Four UAVs allocate eight static tasks with CBBA from Choi, Brunet, and How
(2009). Each UAV builds a bundle of two tasks, orders them by best path
insertion, exchanges winning bids, resolves conflicts using timestamps, and
removes the lost task and all later bundle entries when outbid.

Path score: `S_i = sum(100 * 0.95 ** (arrival_distance / 5))`.

Tasks: entrance, slab edge, laydown yard, wall quality, north/east/west fence,
and material quality at `(12, 10)`. UAVs execute their two-task paths at 12 m.
