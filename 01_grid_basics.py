"""Run first. Predict the neighbors before reading the output."""
from navigation import GRID, START, GOAL, show_grid, neighbors, manhattan

show_grid(GRID)
print('\nCell (row 1, column 1):', GRID[1][1], '(obstacle)')
print('Free neighbors of start:', list(neighbors(GRID, START)))
print('Estimated remaining cost:', manhattan(START, GOAL))
print('Try changing a 0 to a 1, then run again.')
