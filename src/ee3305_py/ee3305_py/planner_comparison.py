"""Read-only comparison of Dijkstra, A* and Theta*."""

from heapq import heappop, heappush
from math import hypot, floor, inf, acos, degrees
from time import perf_counter

from ee3305_py.astar_planner import astar_search
from ee3305_py.theta_planner import theta_search


NEIGHBORS = [
    (1, 0), (1, 1), (0, 1), (-1, 1),
    (-1, 0), (-1, -1), (0, -1), (1, -1),
]


def dijkstra_search(costmap, width, height, resolution,
                    start, goal, max_access_cost=90, stats=None):
    """Match the original planner's 8-neighbour cost rule."""
    if stats is not None:
        stats["nodes_expanded"] = 0

    if width <= 0 or height <= 0 or resolution <= 0:
        return None
    if len(costmap) < width * height:
        return None

    def valid(c, r):
        if not (0 <= c < width and 0 <= r < height):
            return False
        return 0 <= costmap[r * width + c] <= max_access_cost

    if not valid(*start) or not valid(*goal):
        return None

    best = {start: 0.0}
    parent = {}
    heap = [(0.0, 0, start)]
    closed = set()
    sequence = 0

    while heap:
        g, _, current = heappop(heap)
        if current in closed or g > best[current] + 1e-9:
            continue

        closed.add(current)
        if stats is not None:
            stats["nodes_expanded"] += 1

        if current == goal:
            path = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            return path[::-1]

        for dc, dr in NEIGHBORS:
            nxt = (current[0] + dc, current[1] + dr)

            if nxt in closed or not valid(*nxt):
                continue

            cost = costmap[nxt[1] * width + nxt[0]]
            step = hypot(dc, dr) * resolution * (cost + 1)
            tentative = g + step

            if tentative + 1e-9 < best.get(nxt, inf):
                best[nxt] = tentative
                parent[nxt] = current
                sequence += 1
                heappush(heap, (tentative, sequence, nxt))

    return None


def path_metrics(path, resolution):
    length = sum(
        hypot(b[0] - a[0], b[1] - a[1]) * resolution
        for a, b in zip(path, path[1:])
    )

    turns = 0
    for a, b, c in zip(path, path[1:], path[2:]):
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        n1 = hypot(*v1)
        n2 = hypot(*v2)
        if n1 == 0 or n2 == 0:
            continue

        cosine = max(-1.0, min(
            1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
        ))

        if degrees(acos(cosine)) > 5.0:
            turns += 1

    return length, turns


def compare_all(costmap, width, height, resolution,
                origin_x, origin_y, robot_xy, goal_xy,
                max_access_cost=90):

    def to_cell(xy):
        return (
            floor((xy[0] - origin_x) / resolution),
            floor((xy[1] - origin_y) / resolution),
        )

    start = to_cell(robot_xy)
    goal = to_cell(goal_xy)

    results = {}

    for name, search in [
        ("dijkstra", dijkstra_search),
        ("astar", astar_search),
        ("theta", theta_search),
    ]:
        stats = {}
        t0 = perf_counter()

        path = search(
            costmap=costmap,
            width=width,
            height=height,
            resolution=resolution,
            start=start,
            goal=goal,
            max_access_cost=max_access_cost,
            stats=stats,
        )

        elapsed_ms = (perf_counter() - t0) * 1000

        if path is None:
            results[name] = {
                "found": False,
                "planning_ms": round(elapsed_ms, 3),
                "nodes_expanded": stats.get("nodes_expanded", 0),
            }
            continue

        length, turns = path_metrics(path, resolution)

        results[name] = {
            "found": True,
            "planning_ms": round(elapsed_ms, 3),
            "length_m": round(length, 3),
            "nodes_expanded": stats.get("nodes_expanded", 0),
            "waypoints": len(path),
            "turns": turns,
            "path": [
                [
                    origin_x + (c + 0.5) * resolution,
                    origin_y + (r + 0.5) * resolution,
                ]
                for c, r in path
            ],
        }

    return {
        "start": list(robot_xy),
        "goal": list(goal_xy),
        "results": results,
    }
