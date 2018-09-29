import os
import time
import random
import heapq
from math import sqrt
import matplotlib.pyplot as plt
from collections import deque


GRIDSIZE = (10,10);
MAZE = [[0, 1, 0, 1, 0, 0, 0, 0, 0, 0],

        [0, 0, 1, 1, 0, 0, 0, 0, 0, 0],

        [0, 0, 1, 0, 0, 0, 1, 0, 0, 0],

        [1, 0, 1, 0, 0, 0, 0, 0, 0, 0],

        [0, 0, 1, 0, 0, 0, 1, 0, 1, 1],

        [1, 0, 0, 1, 0, 0, 0, 1, 0, 0],

        [0, 0, 1, 0, 0, 0, 1, 1, 0, 1],

        [0, 0, 0, 0, 0, 0, 1, 0, 1, 0],

        [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],

        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]


def plot(maze, GRIDSIZE):
    plt.xlim(0, GRIDSIZE[1]);
    plt.ylim(0, GRIDSIZE[0]);
    ax = plt.axes();
    ax.set_xticks( [i for i in xrange(GRIDSIZE[1])], minor=False);
    ax.set_yticks( [i for i in xrange(GRIDSIZE[0])], minor=False);
    plt.grid(True);
    for i in xrange(GRIDSIZE[0]):
        for j in xrange(GRIDSIZE[1]):
            if maze[i][j] == 1:
                plt.plot( j + 0.5, GRIDSIZE[0] - i - 0.5, 'o', color='black');
            elif maze[i][j] == -1:
                plt.plot( j + 0.5, GRIDSIZE[0] - i - 0.5, 'x', color='blue');
            elif maze[i][j] == 2:
                plt.plot( j + 0.5, GRIDSIZE[0] - i - 0.5, 'o', color='red');
    plt.savefig('Astar_Euclidean.png');

# graph represented as dict; keys = node coord, values = neighbor coord (tuple with 2 num)
def convertMaze(maze):
    height = len(maze)
    width = len(maze[0]) if height else 0
    graph = {(i, j): [] for j in range(width) for i in range(height) if not maze[i][j]}
    for row, col in graph.keys():
        if row < height - 1 and not maze[row + 1][col]:
            graph[(row, col)].append(("Down, ", (row + 1, col)))
            graph[(row + 1, col)].append(("Up, ", (row, col)))
        if col < width - 1 and not maze[row][col + 1]:
            graph[(row, col)].append(("Right, ", (row, col + 1)))
            graph[(row, col + 1)].append(("Left, ", (row, col)))
    return graph


# queue of initial node, check if first in queue is visited, if next is goal, add neighbor to queue
def solveBFS(maze):
    start, goal = (0, 0), (len(maze) - 1, len(maze[0]) - 1)
    queue = deque([("", start)])
    visited = set()
    graph = convertMaze(maze)
    explored = 0;
    while queue:
        path, current = queue.popleft()
        if current == goal:
            return explored, path, visited
        if current in visited:
            continue
        visited.add(current)
        explored += 1;
        for direction, neighbour in graph[current]:
            queue.append((path + direction, neighbour))
    return explored, [], visited;


# bfs with stack
def solveDFS(maze):
    start, goal = (0, 0), (len(maze) - 1, len(maze[0]) - 1)
    stack = deque([("", start)])
    visited = set()
    graph = convertMaze(maze)
    explored = 0;
    while stack:
        path, current = stack.pop()
        if current == goal:
            return explored, path, visited;
        if current in visited:
            continue
        visited.add(current)
        explored += 1
        for direction, neighbour in graph[current]:
            stack.append((path + direction, neighbour))
    return explored, [], visited;


if __name__ == "__main__":

    #start = time.time();
    print("BFS Path:")
    print(solveBFS(MAZE))
    print("DFS Path:")
    print(solveDFS(MAZE))
    #end = time.time();
    #print(end - start);

    print("----------------------------")
