"""Ideal simulated point robot. No hardware or motor interfaces.

World coordinates: x right, y up, heading counterclockwise in radians.
Turn in place, then move straight; this suits our axis-aligned grid route.
"""
from dataclasses import dataclass
from math import atan2, cos, sin, hypot, pi

CELL_SIZE = 1.0  # metres
DT = 0.05        # simulated seconds per update
MAX_SPEED = 0.8  # metres/second
MAX_TURN = 2.0   # radians/second
KP = 4.0        # proportional heading gain, 1/second


def wrap_angle(angle):
    return (angle + pi) % (2 * pi) - pi


def cell_center(cell, rows):
    row, col = cell
    return ((col + 0.5) * CELL_SIZE, (rows - row - 0.5) * CELL_SIZE)


@dataclass
class Robot:
    x: float
    y: float
    theta: float = 0.0


def update_robot(robot, target, dt=DT):
    """Advance one timestep. Return True only once target is reached.

    Heading is aligned to numerical precision before driving. Translation is
    clamped to the remaining distance to prevent overshoot in this ideal model.
    """
    if dt <= 0:
        raise ValueError('dt must be positive.')
    dx, dy = target[0] - robot.x, target[1] - robot.y
    distance = hypot(dx, dy)
    if distance < 1e-9:
        return True
    desired = atan2(dy, dx)
    error = wrap_angle(desired - robot.theta)
    if abs(error) > 1e-6:
        omega = max(-MAX_TURN, min(MAX_TURN, KP * error))
        turn = min(abs(omega * dt), abs(error))
        robot.theta = wrap_angle(robot.theta + (turn if error > 0 else -turn))
        return False
    robot.theta = desired
    step = min(MAX_SPEED * dt, distance)
    robot.x += step * cos(robot.theta)
    robot.y += step * sin(robot.theta)
    return hypot(target[0] - robot.x, target[1] - robot.y) < 1e-9


def simulate(path, rows, max_steps=100000):
    """Headless simulation used by tests and --headless demo."""
    points = [cell_center(cell, rows) for cell in path]
    if not points:
        raise ValueError('Cannot follow an empty path.')
    robot = Robot(*points[0])
    target_index = 1
    for steps in range(max_steps):
        if target_index == len(points):
            return robot, steps
        if update_robot(robot, points[target_index]):
            target_index += 1
    raise RuntimeError('Simulation timed out before reaching the goal.')
