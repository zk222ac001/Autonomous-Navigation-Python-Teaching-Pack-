"""Compare A* and uniform-cost search on the same maps.
For unit costs, uniform-cost search has BFS-equivalent cost ordering.
Do not promise A* always expands fewer cells: ties and maps matter.
"""
from navigation import astar, manhattan, GRID, START, GOAL

for title, grid, start, goal in [
    ('Classroom map', GRID, START, GOAL),
    ('Open room, goal along top row', [[0]*10 for _ in range(10)], (0,0), (0,9)),
]:
    print('\n' + title)
    for name, heuristic in [('Uniform-cost (h=0)', lambda a,b: 0), ('A* Manhattan', manhattan)]:
        path, expanded = astar(grid, start, goal, heuristic)
        cost = None if path is None else len(path)-1
        print(f'{name}: cost={cost}, expansions={len(expanded)}')
