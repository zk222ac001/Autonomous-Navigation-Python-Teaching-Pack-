"""Find a shortest route and inspect how many cells were expanded."""
from navigation import GRID, START, GOAL, astar, show_grid

path, expanded = astar(GRID, START, GOAL)
if path is None:
    print('No route exists. Robot must remain stopped.')
else:
    show_grid(GRID, path)
    print('\nRoute:', path)
    print('Number of moves / cost:', len(path) - 1)
print('Expanded cells:', len(expanded))
print('First five expanded:', expanded[:5])
