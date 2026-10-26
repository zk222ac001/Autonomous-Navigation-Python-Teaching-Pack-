"""Shared grid and A* functions. Coordinates are always (row, column)."""
from heapq import heappush, heappop
from itertools import count

# 0 = free cell, 1 = obstacle. Moves are up/down/left/right, each costing 1.
GRID = [
    [0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 1, 0],
    [1, 1, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
]
START = (0, 0)
GOAL = (4, 5)


def validate_grid(grid):
    if not grid or not grid[0]:
        raise ValueError('Grid must not be empty.')
    if any(len(row) != len(grid[0]) for row in grid):
        raise ValueError('All grid rows must have equal length.')
    if any(cell not in (0, 1) for row in grid for cell in row):
        raise ValueError('Use only 0 (free) and 1 (obstacle).')


def is_free(grid, cell):
    row, col = cell
    return (0 <= row < len(grid) and 0 <= col < len(grid[0])
            and grid[row][col] == 0)


def neighbors(grid, cell):
    """Yield only free cells reachable in one legal move."""
    row, col = cell
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        candidate = (row + dr, col + dc)
        if is_free(grid, candidate):
            yield candidate


def manhattan(a, b):
    """Lower bound on remaining cost for our four-direction unit-cost grid."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal, heuristic=manhattan):
    """Return (path, expansion_order). None path means unreachable.

    Invalid maps/endpoints raise ValueError. The default heuristic is consistent.
    Improved routes can reopen nodes; outdated heap entries are skipped.
    A custom heuristic must be admissible, nonnegative and zero at the goal
    to retain the optimality guarantee in this finite unit-cost setting.
    """
    validate_grid(grid)
    if not is_free(grid, start) or not is_free(grid, goal):
        raise ValueError('Start and goal must be in-bounds free cells.')

    frontier = []                       # Min-heap: next candidate has lowest f.
    serial = count()                    # FIFO tie-breaker for equal f scores.
    g_score = {start: 0}                 # Best known cost from start.
    came_from = {}                       # Parent pointers for route recovery.
    expanded = []
    heappush(frontier, (heuristic(start, goal), next(serial), 0, start))

    while frontier:
        _, _, popped_g, current = heappop(frontier)
        if popped_g != g_score[current]:
            continue                    # A better entry replaced this old one.
        expanded.append(current)
        if current == goal:             # Stop on removal, not first discovery.
            path = [goal]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            path.reverse()
            return path, expanded

        for neighbor in neighbors(grid, current):
            tentative_g = popped_g + 1
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heappush(frontier, (f_score, next(serial), tentative_g, neighbor))

    return None, expanded


def show_grid(grid, path=(), start=START, goal=GOAL):
    route = set(path or [])
    for r, row in enumerate(grid):
        symbols = []
        for c, value in enumerate(row):
            cell = (r, c)
            symbol = '#' if value else '.'
            if cell in route:
                symbol = '*'
            if cell == start:
                symbol = 'S'
            if cell == goal:
                symbol = 'G'
            symbols.append(symbol)
        print(' '.join(symbols))
