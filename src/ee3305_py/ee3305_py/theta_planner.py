"""Theta* any-angle path planning for the EE3305 occupancy grid."""

from heapq import heappush, heappop
from math import hypot, inf


def theta_search(costmap, width, height, resolution,
                 start, goal, max_access_cost=100):

    if width <= 0 or height <= 0 or resolution <= 0:
        return None
    if len(costmap) < width * height:
        return None

    def traversable(c, r):
        if not (0 <= c < width and 0 <= r < height):
            return False
        cost = costmap[r * width + c]
        return 0 <= cost <= max_access_cost

    if not traversable(*start) or not traversable(*goal):
        return None
    if start == goal:
        return [start]

    def heuristic(node):
        return resolution * hypot(
            goal[0] - node[0], goal[1] - node[1]
        )

    def line_cost(a, b):
        """Weighted line traversal with obstacle and corner checks."""
        x, y = a
        dx = b[0] - x
        dy = b[1] - y

        if dx == 0 and dy == 0:
            return 0.0

        sx = (dx > 0) - (dx < 0)
        sy = (dy > 0) - (dy < 0)

        tx = 0.5 / abs(dx) if dx else inf
        ty = 0.5 / abs(dy) if dy else inf
        dtx = 1.0 / abs(dx) if dx else inf
        dty = 1.0 / abs(dy) if dy else inf

        distance = resolution * hypot(dx, dy)
        previous_t = 0.0
        total_cost = 0.0

        while True:
            if not traversable(x, y):
                return inf

            next_t = min(tx, ty, 1.0)
            cell_cost = costmap[y * width + x]

            total_cost += (
                (next_t - previous_t)
                * distance
                * (cell_cost + 1)
            )

            if next_t >= 1.0:
                break

            if abs(tx - ty) < 1e-12:
                # Prevent cutting diagonally through blocked corners.
                if not traversable(x + sx, y):
                    return inf
                if not traversable(x, y + sy):
                    return inf
                x += sx
                y += sy
                tx += dtx
                ty += dty
            elif tx < ty:
                x += sx
                tx += dtx
            else:
                y += sy
                ty += dty

            previous_t = next_t

        return total_cost

    neighbors = [
        (1, 0), (1, 1), (0, 1), (-1, 1),
        (-1, 0), (-1, -1), (0, -1), (1, -1)
    ]

    g_score = {start: 0.0}
    parent = {start: start}
    closed = set()
    heap = [(heuristic(start), 0, start)]
    sequence = 0

    while heap:
        f, _, current = heappop(heap)

        if current in closed:
            continue
        if f > g_score[current] + heuristic(current) + 1e-9:
            continue

        if current == goal:
            path = [current]
            while current != start:
                current = parent[current]
                path.append(current)
            return path[::-1]

        closed.add(current)
        c, r = current

        for dc, dr in neighbors:
            neighbor = (c + dc, r + dr)

            if neighbor in closed or not traversable(*neighbor):
                continue

            # Safe neighbor transition.
            direct_cost = line_cost(current, neighbor)
            if direct_cost == inf:
                continue

            best_parent = current
            best_g = g_score[current] + direct_cost

            # Theta*: try connecting through current's parent.
            ancestor = parent[current]
            if ancestor != current:
                shortcut_cost = line_cost(ancestor, neighbor)

                if shortcut_cost < inf:
                    shortcut_g = (
                        g_score[ancestor] + shortcut_cost
                    )

                    if shortcut_g < best_g:
                        best_g = shortcut_g
                        best_parent = ancestor

            if best_g + 1e-9 < g_score.get(neighbor, inf):
                g_score[neighbor] = best_g
                parent[neighbor] = best_parent

                sequence += 1
                heappush(
                    heap,
                    (
                        best_g + heuristic(neighbor),
                        sequence,
                        neighbor
                    )
                )

    return None
