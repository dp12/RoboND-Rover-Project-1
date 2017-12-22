# -----------
# This code is adapted from the Udacity CS373 A* implementation
#
# User Instructions:
#
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
#
# If there is no path from init to goal,
# the function should return the string 'fail'
# ----------
# DONE: use the heapq implementation shown here:
# DONE: return a list of waypoints
# DONE: is grid sense the same as the rover map orientation
# - doesn't even matter, as long as x,y waypoint pairs are returned correctly
# DONE: add the four diagonal directions
# DONE: fix astar to get the optimal path
# profile Python code with line_profiler @profile
# https://stackoverflow.com/questions/3927628/how-can-i-profile-python-code-line-by-line
# https://stackoverflow.com/questions/17328506/priorityqueue-is-very-slow
from heapq import *
from perception import Cell
import numpy as np

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [-1, -1], # go up-left
         [-1, 1 ], # go up-right
         [ 0, -1], # go left
         [ 1, -1], # go down-left
         [ 1, 0 ], # go down
         [ 1, 1 ], # go down-right
         [ 0, 1 ]] # go right

delta_name = ['^', '\^', '^/', '<', '/v', 'v', 'v\\', '>']

def euclidean_heuristic(start, goal):
    return np.linalg.norm(np.array(goal) - np.array(start))

def astar(grid, init, goal, cost):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    # h = heuristic[x][y]
    h = euclidean_heuristic([x, y], goal)
    f = g + h

    open = []
    heappush(open, (f, g, h, x, y))

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0

    while not found and not resign:
        if len(open) == 0:
            resign = True
            print("A* failed")
            return None
        else:
            next = heappop(open)
            x = next[3]
            y = next[4]
            g = next[1]
            expand[x][y] = count
            count += 1

            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == Cell.FREE:
                            g2 = g + cost
                            # h2 = heuristic[x2][y2]
                            h2 = euclidean_heuristic([x2, y2], goal)
                            f2 = g2 + h2
                            heappush(open, (f2, g2, h2, x2, y2))
                            closed[x2][y2] = 1
                            action[x2][y2] = i

    # Walk backwards to find the waypoints from the actions
    # policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    # policy[x][y] = '*'
    waypoints = []
    x = goal[0]
    y = goal[1]
    while x != init[0] or y != init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        # policy[x2][y2] = delta_name[action[x][y]]
        waypoints.append([x2,y2])
        x = x2
        y = y2

    return waypoints
    # return expand

# print search(grid, init, goal, cost, heuristic)
