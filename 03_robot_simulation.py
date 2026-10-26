"""Animated robot: python 03_robot_simulation.py
Terminal only: python 03_robot_simulation.py --headless
"""
import argparse
from math import cos, sin
from navigation import GRID, START, GOAL, astar
from controller import Robot, cell_center, update_robot, simulate, DT, CELL_SIZE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    path, expanded = astar(GRID, START, GOAL)
    if path is None:
        print('No route. Robot stopped.')
        return
    if args.headless:
        robot, steps = simulate(path, len(GRID))
        print(f'Goal reached: x={robot.x:.2f}, y={robot.y:.2f}; updates={steps}')
        return

    try:
        import tkinter as tk
        root = tk.Tk()
    except Exception as exc:
        # A missing desktop/display can also prevent Tk from opening a window.
        print(f'Cannot open graphical display: {exc}')
        print('Use: python 03_robot_simulation.py --headless')
        return
    root.title('A* classroom robot — blue route, orange robot')
    scale = 82
    rows, cols = len(GRID), len(GRID[0])
    canvas = tk.Canvas(root, width=cols * scale, height=rows * scale, bg='white')
    canvas.pack()
    status = tk.Label(root, text='Press Start to follow the planned route.')
    status.pack(pady=8)
    for r, row in enumerate(GRID):
        for c, value in enumerate(row):
            fill = '#243449' if value else ('#def0ed' if (r,c) in expanded else '#f8fafc')
            canvas.create_rectangle(c*scale, r*scale, (c+1)*scale, (r+1)*scale,
                                    fill=fill, outline='#aab5bf')
            canvas.create_text(c*scale+23, r*scale+13, text=f'{r},{c}', fill='#718096')
    for a, b in zip(path, path[1:]):
        canvas.create_line((a[1]+.5)*scale, (a[0]+.5)*scale,
                           (b[1]+.5)*scale, (b[0]+.5)*scale, fill='#2586d4', width=5)
    for label, cell in [('S', START), ('G', GOAL)]:
        canvas.create_text((cell[1]+.5)*scale, (cell[0]+.5)*scale-24,
                           text=label, font=('Arial', 15, 'bold'))
    robot = Robot(*cell_center(path[0], rows))
    points = [cell_center(cell, rows) for cell in path]
    body = canvas.create_oval(0, 0, 0, 0, fill='#ffb347', outline='#9c5700', width=2)
    arrow = canvas.create_line(0, 0, 0, 0, fill='#111827', width=3, arrow=tk.LAST)
    index = 1
    running = False
    after_id = None

    def draw():
        x, y = robot.x / CELL_SIZE * scale, (rows - robot.y / CELL_SIZE) * scale
        canvas.coords(body, x-13, y-13, x+13, y+13)
        canvas.coords(arrow, x, y, x+23*cos(robot.theta), y-23*sin(robot.theta))

    def tick():
        nonlocal index, running, after_id
        after_id = None
        if not running:
            return
        if index == len(points):
            running = False
            status.config(text='Goal reached. Close and rerun to restart.')
            return
        if update_robot(robot, points[index]):
            index += 1
        draw()
        status.config(text=f'Waypoint {min(index, len(points)-1)}/{len(points)-1} | '
                           f'x={robot.x:.2f} m, y={robot.y:.2f} m')
        after_id = root.after(round(DT * 1000), tick)

    def toggle():
        nonlocal running, after_id
        if index == len(points):
            return
        if after_id is not None:
            root.after_cancel(after_id)
            after_id = None
        running = not running
        button.config(text='Pause' if running else 'Resume')
        if running:
            tick()

    button = tk.Button(root, text='Start', command=toggle)
    button.pack(pady=5)
    draw()
    root.mainloop()


if __name__ == '__main__':
    main()
