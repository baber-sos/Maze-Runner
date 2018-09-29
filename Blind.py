import os
import time
from collections import deque


# graph of dictionaries where key = current coord, values = neighbor coord; append directions per access
def convertMaze(maze):
    length = len(maze)
    width = len(maze[0]) if length else 0
    graph = {(i, j): [] for j in range(width) for i in range(length) if not maze[i][j]}
    for row, col in graph.keys():
        if row < length - 1 and not maze[row + 1][col]:
            graph[(row, col)].append(("Down, ", (row + 1, col)))
            graph[(row + 1, col)].append(("Up, ", (row, col)))
        if col < width - 1 and not maze[row][col + 1]:
            graph[(row, col)].append(("Right, ", (row, col + 1)))
            graph[(row, col + 1)].append(("Left, ", (row, col)))
    return graph


# queue of initial node -> check if next node is goal node, if not add neighbor nodes to queue and repeat
def solveBFS(maze):
    s, g = (0, 0), (len(maze) - 1, len(maze[0]) - 1)
    queue = deque([("", s)])
    visited = set()
    graph = convertMaze(maze)
    while queue:
        path, current = queue.popleft()
        if current == g:
            return path
        if current in visited:
            continue
        visited.add(current)
        for direction, neighbour in graph[current]:
            queue.append((path + direction, neighbour))
    return "No solvable path"


# bfs but stack
def solveDFS(maze):
    start, goal = (0, 0), (len(maze) - 1, len(maze[0]) - 1)
    stack = deque([("", start)])
    visited = set()
    graph = convertMaze(maze)
    while stack:
        path, current = stack.pop()
        if current == goal:
            return path
        if current in visited:
            continue
        visited.add(current)
        for direction, neighbour in graph[current]:
            stack.append((path + direction, neighbour))
    return "No solvable path"


MAZE = [
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1],
    [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1],
    [1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
]


if __name__ == "__main__":

    #MAZE.find_path_bfs;
    start = time.time();
    #find_path_bfs(MAZE)


    #start = time.time();
    print("BFS Path:")
    print(solveBFS(MAZE))
    print("DFS Path:")
    print(solveDFS(MAZE))
    end = time.time();
    print(end - start);

    print("----------------------------")

# num of explored nodes
# path taken
# maze generated