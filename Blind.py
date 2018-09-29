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


def plot_v2(maze, name, path, visited):
    print path[0];
    directions = {'Down': (1, 0),
            'Up': (-1, 0),
            'Right': (0, 1),
            'Left': (0, -1)};
    print directions;
    GRIDSIZE = (len(maze[0]), len(maze));
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
    start = [0, 0];
    if path == []:
        plt.savefig(name);
        return;
    for coord in visited:
        plt.plot( coord[1] + 0.5, GRIDSIZE[0] - coord[0] - 0.5, 'x', color='blue');
    for dir in path:
        print 'cuurent dir: ', dir;
        plt.plot( start[1] + 0.5, GRIDSIZE[0] - start[0] - 0.5, 'o', color='red');
        start[0] += directions[dir][0]; start[1] += directions[dir][1]; 
    plt.plot( start[1] + 0.5, GRIDSIZE[0] - start[0] - 0.5, 'o', color='red');
    plt.savefig(name);

# graph represented as dict; keys = node coord, values = neighbor coord (tuple with 2 num)
def convertMaze(maze):
    height = len(maze)
    width = len(maze[0]) if height else 0
    graph = {(i, j): [] for j in range(width) for i in range(height) if not maze[i][j]}
    for row, col in graph.keys():
        if row < height - 1 and not maze[row + 1][col]:
            graph[(row, col)].append(("Down", (row + 1, col)))
            graph[(row + 1, col)].append(("Up", (row, col)))
        if col < width - 1 and not maze[row][col + 1]:
            graph[(row, col)].append(("Right", (row, col + 1)))
            graph[(row, col + 1)].append(("Left", (row, col)))
    return graph


# queue of initial node, check if first in queue is visited, if next is goal, add neighbor to queue
def solveBFS(maze):
    start, goal = (0, 0), (len(maze) - 1, len(maze[0]) - 1)
    queue = deque([([], start)])
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
            queue.append((path + [direction], neighbour))
    return explored, [], visited;


# bfs with stack
def solveDFS(maze):
    start, goal = (0, 0), (len(maze) - 1, len(maze[0]) - 1)
    stack = deque([([], start)])
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
            stack.append((path + [direction], neighbour))
    return explored, [], visited;


if __name__ == "__main__":

    #start = time.time();
    print("BFS Path:")
    explored_number, path, visited = solveBFS(MAZE);
    print path
    plot_v2(MAZE, 'bfs.png', path, visited);
    print("DFS Path:")
    explored_number, path, visited = solveDFS(MAZE);
    plot_v2(MAZE, 'dfs.png', path, visited);
    #end = time.time();
    #print(end - start);

    print("----------------------------")
