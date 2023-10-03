import unittest

from board import *
from solve import *


class TestGetPath(unittest.TestCase):

    def test_get_path_single_state(self):
        # Create a single state and check if the path contains only that state
        board = Board("TestBoard", 3, 3, [], [], [], [])
        initial_state = State(board, heuristic_zero, 0, 0)
        path = get_path(initial_state)
        self.assertEqual(len(path), 1)
        self.assertEqual(path[0], initial_state)

    def test_get_path_multiple_states(self):
        # Create a path of states and check if the path is in the correct order
        board = Board("TestBoard", 3, 3, [], [], [], [])
        initial_state = State(board, heuristic_zero, 0, 0)
        state1 = State(board, heuristic_zero, 1, 1, parent=initial_state)
        state2 = State(board, heuristic_zero, 2, 2, parent=state1)
        path = get_path(state2)
        self.assertEqual(len(path), 3)
        self.assertEqual(path[0], initial_state)
        self.assertEqual(path[1], state1)
        self.assertEqual(path[2], state2)

    def test_get_path_no_parent(self):
        # Create a state with no parent, the path should contain only that state
        board = Board("TestBoard", 3, 3, [], [], [], [])
        state = State(board, heuristic_zero, 0, 0)
        path = get_path(state)
        self.assertEqual(len(path), 1)
        self.assertEqual(path[0], state)

class TestIsGoal(unittest.TestCase):

    def test_goal_state(self):
        # Create a board where all boxes are on storage points
        board = Board("Test Board", 5, 5, [], {(2, 2), (3, 3)}, {(2, 2), (3, 3)}, set())
        state = State(board, None, 0, 0)
        self.assertTrue(is_goal(state))

    def test_not_goal_state(self):
        # Create a board where not all boxes are on storage points
        board = Board("Test Board", 5, 5, [], {(2, 2), (3, 3)}, {(2, 2)}, set())
        state = State(board, None, 0, 0)
        self.assertFalse(is_goal(state))

    def test_empty_board(self):
        # Create an empty board (no boxes)
        board = Board("Test Board", 5, 5, [], set(), set(), set())
        state = State(board, None, 0, 0)
        self.assertTrue(is_goal(state))

    def test_mixed_state(self):
        # Create a board with some boxes on storage points and some not
        board = Board("Test Board", 5, 5, [], {(2, 2), (3, 3)}, {(3, 3)}, set())
        state = State(board, None, 0, 0)
        self.assertFalse(is_goal(state))
class TestGetSuccessors(unittest.TestCase):

    def test_successors_basic(self):
        # Test with a simple board and single robot
        board = Board("Test Board", 4, 3, [(1, 1)], frozenset(), frozenset(), frozenset())
        state = State(board, heuristic_zero, 0, 0)
        successors = get_successors(state)
        expected_positions = [(0, 1), (2, 1), (1, 0), (1, 2)]
        actual_positions = [(s.board.robots[0][0], s.board.robots[0][1]) for s in successors]
        self.assertEqual(sorted(expected_positions), sorted(actual_positions))

    def test_successors_obstacles(self):
        # Test with obstacles blocking robot movement
        board = Board("Test Board", 4, 4, [(1, 1)], frozenset(), frozenset(), {(1, 2), (2, 1)})
        state = State(board, heuristic_zero, 0, 0)
        successors = get_successors(state)
        expected_positions = [(0, 1), (1, 0)]
        actual_positions = [(s.board.robots[0][0], s.board.robots[0][1]) for s in successors]
        self.assertEqual(sorted(expected_positions), sorted(actual_positions))

    def test_successors_multiple_robots(self):
        # Test with multiple robots
        board = Board("Test Board", 4, 4, [(1, 1), (2, 2)], frozenset(), frozenset(), frozenset())
        state = State(board, heuristic_zero, 0, 0)
        successors = get_successors(state)
        expected_positions = [(0, 1), (2, 2), (1, 0), (3, 1), (2, 1)]
        actual_positions = [(s.board.robots[0][0], s.board.robots[0][1]) for s in successors]
        self.assertEqual(sorted(expected_positions), sorted(actual_positions))


if __name__ == '__main__':
    unittest.main()
