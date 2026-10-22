
"""Independent A* search for the EE3305 occupancy-grid planner.

This module contains the A* algorithm only.
The existing planner.py remains responsible for ROS2 communication,
planner selection, and publishing the selected path.
"""

from heapq import heappush, heappop
from math import hypot, sqrt, inf


def astar_search(
    costmap,
    width,
    height,
    resolution,
    start,
    goal,
    max_access_cost=100,
):
    """
    Find a minimum-cost path through an 8-connected occupancy grid.

    Args:
        costmap: Flattened OccupancyGrid data, indexed as row * width + col.
        width, height: Grid dimensions in cells.
        resolution: Cell size in metres.
        start, goal: (column, row) tuples.
        max_access_cost: Highest permitted cell cost.

    Returns:
        List of (column, row) coordinates, start to goal.
        Returns None when no valid path exists.
    """

    def in_bounds(c, r):
        return 0 <= c < width and 0 <= r < height

    def traversable(c, r):
        if not in_bounds(c, r):
            return False

        cost = costmap[r * width + c]
        return 0 <= cost <= max_access_cost

    def heuristic(c, r):
        # Octile distance for an 8-connected grid.
        dx = abs(goal[0] - c)
        dy = abs(goal[1] - r)

        return resolution * (
            max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)
        )

    if resolution <= 0 or width <= 0 or height <= 0:
        return None

    if len(costmap) < width * height:
        return None

    if not traversable(*start) or not traversable(*goal):
        return None

    if start == goal:
        return [start]

    neighbors = [
        (1, 0), (1, 1), (0, 1), (-1, 1),
        (-1, 0), (-1, -1), (0, -1), (1, -1),
    ]

    # Entries: (estimated total cost, sequence, cell).
    open_heap = []
    sequence = 0

    heappush(open_heap, (heuristic(*start), sequence, start))

    g_score = {start: 0.0}
    came_from = {}
    closed = set()

    while open_heap:
        current_f, _, current = heappop(open_heap)

        if current in closed:
            continue

        # Ignore outdated heap entries.
        if current_f > g_score[current] + heuristic(*current) + 1e-9:
            continue

        if current == goal:
            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()
            return path

        closed.add(current)

        c, r = current

        for dc, dr in neighbors:
            nc, nr = c + dc, r + dr
            neighbor = (nc, nr)

            if neighbor in closed or not traversable(nc, nr):
                continue

            # Match the existing Dijkstra cost calculation.
            step_distance = hypot(
                dc * resolution,
                dr * resolution,
            )

            cell_cost = costmap[nr * width + nc]
            step_cost = step_distance * (cell_cost + 1)

            tentative_g = g_score[current] + step_cost

            if tentative_g < g_score.get(neighbor, inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current

                sequence += 1
                heappush(
                    open_heap,
                    (
                        tentative_g + heuristic(nc, nr),
                        sequence,
                        neighbor,
                    ),
                )

    return None
