"""Student worksheet code: complete the two functions, then run this file."""
from navigation import GRID


def student_manhattan(a, b):
    # TODO: absolute row difference + absolute column difference.
    raise NotImplementedError('Complete student_manhattan')


def student_neighbors(grid, cell):
    # TODO: return a list of legal up/down/left/right neighboring cells.
    # Reject negative indices BEFORE accessing the grid!
    raise NotImplementedError('Complete student_neighbors')


if __name__ == '__main__':
    assert student_manhattan((0,0), (4,5)) == 9
    assert student_manhattan((2,3), (2,3)) == 0
    assert set(student_neighbors(GRID, (0,0))) == {(1,0), (0,1)}
    assert set(student_neighbors(GRID, (2,2))) == {(2,1), (2,3), (3,2)}
    print('Starter checks passed. Now add your own edge-case checks.')
