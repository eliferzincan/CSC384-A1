############################################################
## CSC 384, Intro to AI, University of Toronto.
## Assignment 1 Starter Code
## v1.0
############################################################

import copy
from typing import List
import heapq
from heapq import heappush, heappop
import time
import argparse
import math  # for infinity

from board import *


def is_goal(state):
    """
    Returns True if the state is the goal state and False otherwise.

    :param state: the current state.
    :type state: State
    :return: True or False
    :rtype: bool
    """
    box_positions = state.board.boxes
    storage_positions = state.board.storage
    for box in box_positions:
        if box not in storage_positions:
            return False
    return True


def get_path(state):
    """
    Return a list of states containing the nodes on the path 
    from the initial state to the given state in order.

    :param state: The current state.
    :type state: State
    :return: The path.
    :rtype: List[State]
    """
    parents = [state]
    curr_state = state
    while curr_state.parent is not None:
        parents.insert(0, curr_state.parent)
        curr_state = curr_state.parent
    return parents


def get_successors(state):
    """
    Return a list containing the successor states of the given state.
    The states in the list may be in any arbitrary order.

    :param state: The current state.
    :type state: State
    :return: The list of successor states.
    :rtype: List[State]
    """
    successors = []
    visited_states = set()

    moves = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    for robot_index, (robot_x, robot_y) in enumerate(state.board.robots):
        for move_x, move_y in moves:
            new_x, new_y = robot_x + move_x, robot_y + move_y
            new_robot_position = (new_x, new_y)

            if (
                    0 <= new_x < state.board.width
                    and 0 <= new_y < state.board.height
                    and new_robot_position not in state.board.obstacles
            ):
                new_board = copy.deepcopy(state.board)
                new_board.robots[robot_index] = new_robot_position
                new_state = State(
                    board=new_board,
                    hfn=state.hfn,
                    f=state.f,
                    depth=state.depth + 1,
                    parent=state,
                )

                if new_state not in visited_states:
                    successors.append(new_state)
                    visited_states.add(new_state)

    return successors


def dfs(init_board):
    """
    Run the DFS algorithm given an initial board.

    If the function finds a goal state, it returns a list of states representing
    the path from the initial state to the goal state in order and the cost of
    the solution found.
    Otherwise, it returns am empty list and -1.

    :param init_board: The initial board.
    :type init_board: Board
    :return: (the path to goal state, solution cost)
    :rtype: List[State], int
    """
    # Create a stack to store states (newest state added is removed first)
    frontier = [State(board=init_board, hfn=heuristic_zero, f=0, depth=0)]

    # Create a set to store visited states
    visited = set()

    while frontier:
        current_state = frontier.pop()

        # Check if the current state is the goal state
        if is_goal(current_state):
            # If yes, reconstruct the path and return it
            path = get_path(current_state)
            return path, len(path) - 1  # Solution cost is the length of the path - 1

        # Mark the current state as visited
        visited.add(current_state.id)

        # Generate successor states
        successors = get_successors(current_state)

        for successor in successors:
            # Check if the successor state has not been visited
            if successor.id not in visited:
                frontier.append(successor)

    # If no goal state is found, return an empty path and -1
    return [], -1


def a_star(init_board, hfn):
    """
    Run the A_star search algorithm given an initial board and a heuristic function.

    If the function finds a goal state, it returns a list of states representing
    the path from the initial state to the goal state in order and the cost of
    the solution found.
    Otherwise, it returns am empty list and -1.

    :param init_board: The initial starting board.
    :type init_board: Board
    :param hfn: The heuristic function.
    :type hfn: Heuristic (a function that consumes a Board and produces a numeric heuristic value)
    :return: (the path to goal state, solution cost)
    :rtype: List[State], int
    """

    raise NotImplementedError


def heuristic_basic(board):
    """
    Returns the heuristic value for the given board
    based on the Manhattan Distance Heuristic function.

    Returns the sum of the Manhattan distances between each box 
    and its closest storage point.

    :param board: The current board.
    :type board: Board
    :return: The heuristic value.
    :rtype: int
    """

    raise NotImplementedError


def heuristic_advanced(board):
    """
    An advanced heuristic of your own choosing and invention.

    :param board: The current board.
    :type board: Board
    :return: The heuristic value.
    :rtype: int
    """

    raise NotImplementedError


def solve_puzzle(board: Board, algorithm: str):
    """
    Solve the given puzzle using the given type of algorithm.

    :param algorithm: the search algorithm
    :type algorithm: str
    :param hfn: The heuristic function
    :type hfn: Optional[Heuristic]

    :return: the path from the initial state to the goal state
    :rtype: List[State]
    """

    print("Initial board")
    board.display()

    time_start = time.time()

    if algorithm == 'a_star':
        print("Executing A* search")
        path, step = a_star(board, hfn)
    elif algorithm == 'dfs':
        print("Executing DFS")
        path, step = dfs(board, hfn)
    else:
        raise NotImplementedError

    time_end = time.time()
    time_elapsed = time_end - time_start

    if not path:

        print('No solution for this puzzle')
        return []

    else:

        print('Goal state found: ')
        path[-1].board.display()

        print('Solution is: ')

        counter = 0
        while counter < len(path):
            print(counter + 1)
            path[counter].board.display()
            print()
            counter += 1

        print('Solution cost: {}'.format(step))
        print('Time taken: {:.2f}s'.format(time_elapsed))

        return path


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--inputfile",
        type=str,
        required=True,
        help="The file that contains the puzzle."
    )
    parser.add_argument(
        "--outputfile",
        type=str,
        required=True,
        help="The file that contains the solution to the puzzle."
    )
    parser.add_argument(
        "--algorithm",
        type=str,
        required=True,
        choices=['a_star', 'dfs'],
        help="The searching algorithm."
    )
    parser.add_argument(
        "--heuristic",
        type=str,
        required=False,
        default=None,
        choices=['zero', 'basic', 'advanced'],
        help="The heuristic used for any heuristic search."
    )
    args = parser.parse_args()

    # set the heuristic function
    heuristic = heuristic_zero
    if args.heuristic == 'basic':
        heuristic = heuristic_basic
    elif args.heuristic == 'advanced':
        heuristic = heuristic_advanced

    # read the boards from the file
    board = read_from_file(args.inputfile)

    # solve the puzzles
    path = solve_puzzle(board, args.algorithm, heuristic)

    # save solution in output file
    outputfile = open(args.outputfile, "w")
    counter = 1
    for state in path:
        print(counter, file=outputfile)
        print(state.board, file=outputfile)
        counter += 1
    outputfile.close()
