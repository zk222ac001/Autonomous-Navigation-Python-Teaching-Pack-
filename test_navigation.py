"""Run: python -m unittest -v. BFS is an independent shortest-path oracle."""
import random
import unittest
from collections import deque
from math import hypot, pi
from navigation import astar, GRID, START, GOAL, manhattan, is_free
from controller import simulate, cell_center, wrap_angle, Robot, update_robot


def bfs_cost(grid, start, goal):
    queue = deque([(start, 0)])
    seen = {start}
    while queue:
        (r, c), cost = queue.popleft()
        if (r, c) == goal:
            return cost
        for nr, nc in [(r-1,c), (r+1,c), (r,c-1), (r,c+1)]:
            if (0 <= nr < len(grid) and 0 <= nc < len(grid[0])
                    and grid[nr][nc] == 0 and (nr,nc) not in seen):
                seen.add((nr,nc))
                queue.append(((nr,nc), cost+1))
    return None


class NavigationTests(unittest.TestCase):
    def test_default_route(self):
        path, _ = astar(GRID, START, GOAL)
        self.assertEqual(len(path)-1, 9)
        self.assertEqual(path[0], START)
        self.assertEqual(path[-1], GOAL)

    def test_same_cell(self):
        self.assertEqual(astar([[0]], (0,0), (0,0))[0], [(0,0)])

    def test_unreachable(self):
        self.assertIsNone(astar([[0,1,0]], (0,0), (0,2))[0])

    def test_invalid_inputs(self):
        for grid, start, goal in [([], (0,0), (0,0)), ([[0],[0,0]],(0,0),(0,0)),
                                  ([[0,2]],(0,0),(0,1)), ([[0,1]],(0,0),(0,1)),
                                  ([[0]],(-1,0),(0,0))]:
            with self.assertRaises(ValueError):
                astar(grid, start, goal)

    def test_random_maps_against_bfs(self):
        rng = random.Random(42)
        for trial in range(150):
            grid = [[int(rng.random() < .30) for _ in range(9)] for _ in range(7)]
            grid[0][0] = grid[-1][-1] = 0
            path, _ = astar(grid, (0,0), (6,8))
            expected = bfs_cost(grid, (0,0), (6,8))
            self.assertEqual(None if path is None else len(path)-1, expected)
            if path:
                self.assertTrue(all(is_free(grid, p) for p in path))
                self.assertTrue(all(manhattan(a,b) == 1 for a,b in zip(path,path[1:])))

    def test_controller_all_headings(self):
        for target in [(1,0), (-1,0), (0,1), (0,-1)]:
            robot = Robot(0,0,pi/4)
            for _ in range(2000):
                if update_robot(robot, target):
                    break
            self.assertLess(hypot(robot.x-target[0], robot.y-target[1]), 1e-8)
        self.assertAlmostEqual(wrap_angle(3*pi), -pi)

    def test_controller_route(self):
        path, _ = astar(GRID, START, GOAL)
        robot, _ = simulate(path, len(GRID))
        x, y = cell_center(GOAL, len(GRID))
        self.assertLess(hypot(robot.x-x, robot.y-y), 1e-8)


if __name__ == '__main__':
    unittest.main()
