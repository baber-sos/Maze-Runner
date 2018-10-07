import random
import heapq
from math import sqrt
import matplotlib.pyplot as plt
import time
from collections import deque


#probability of obstacles
def generate_maze(probability, GRIDSIZE):
    maze = [[1 if random.random() > (1-probability) else 0 \
        for x in xrange(GRIDSIZE[1])] for i in xrange(GRIDSIZE[0])];
    maze[0][0] = 0;
    maze[GRIDSIZE[1]-1][GRIDSIZE[0]-1] = 0;
    return maze;

#this will be a fringe of threee tuples
#(current_location, estimated_total_cost, actual_cost,  prev_location)
#cost will be the negative of original coset since heapq is a min heap.

def return_neighbors(coordinates, maze, GRIDSIZE):
    neighbors = [];
    incr = [[1, 0], [-1, 0], [0, 1], [0, -1]];
    for val in incr:
        temp = (coordinates[0] + val[0], coordinates[1]+val[1]);
        if temp[0] >= 0 and temp[1] >= 0 and temp[0] < GRIDSIZE[0] and temp[1] < GRIDSIZE[1] \
                and maze[temp[0]][temp[1]] != 1 and maze[temp[0]][temp[1]] != -1:
                    neighbors.append(temp);

    return neighbors;
def plot(maze, GRIDSIZE, name):
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
    plt.savefig(name);
    plt.gcf().clear();

# return types is (num_explored, path_itself, modified_maze_for_plotting);
def astar(maze, GRIDSIZE):
#    for i in xrange(GRIDSIZE[0]):
#        for j in xrange(GRIDSIZE[1]):
#            if maze[i][j] == 1:
#                plt.plot( j + 0.5, GRIDSIZE[0] - i - 0.5, 'o');
#    plt.plot(0.5, GRIDSIZE[0] - 0.5, 'x', color='r');
#    plt.plot(GRIDSIZE[1] - 0.5, 0.5, 'x', color='r');
    #plot(maze, GRIDSIZE);
    if maze[0][0] == 1:
        return 0, [], maze, 0;
    fringe = [ (sqrt( ( float(GRIDSIZE[0]-1)**2 + float(GRIDSIZE[1]-1)**2) ), (0, 0), 0.0, (-1, -1)) ];
    max_size = 1; current_size = 1;
    goal = (GRIDSIZE[0]-1, GRIDSIZE[1]-1);
    path = [ [(-1, -1) for j in xrange(GRIDSIZE[0])] for i in xrange(GRIDSIZE[1])]; #will be filled later
    maze[0][0] = -1;
    explored = 0;
    while len(fringe) != 0:
        current = heapq.heappop(fringe);
        current_size -= 1;
        explored += 1;
        path[current[1][0]][current[1][1]] = current[-1];
#        plt.plot(current[1][1] + 0.5, GRIDSIZE[0] - current[1][0] - 0.5, 'x', color='blue');
        if current[1] == goal:
            break;
        else:
            for child in return_neighbors(current[1], maze, GRIDSIZE):
                current_size += 1;
                heuristic = sqrt(((child[0]-(GRIDSIZE[0]-1))**2) + ((child[1]-(GRIDSIZE[1]-1))**2))
                actual_cost = current[2] + 1;
                maze[child[0]][child[1]] = -1;
                heapq.heappush(fringe, (heuristic + actual_cost, child, actual_cost, current[1]));
            if current_size > max_size:
                max_size = current_size;
    ##backtrack the path here starting from endpoint
    if current[1] != goal:
        return explored, [], maze, max_size;
    current_node = goal;
    shortest_path = [];
    while current_node != (-1, -1):
        maze[current_node[0]][current_node[1]] = 2;
        shortest_path.append(current_node);
#        plt.plot(current_node[1] + 0.5, GRIDSIZE[0] - current_node[0] - 0.5, 'o', color='red');
        current_node = path[current_node[0]][current_node[1]];
#    plt.show();
    return explored, shortest_path, maze, max_size;

def astar_man(maze, GRIDSIZE):
#    for i in xrange(GRIDSIZE[0]):
#        for j in xrange(GRIDSIZE[1]):
#            if maze[i][j] == 1:
#                plt.plot( j + 0.5, GRIDSIZE[0] - i - 0.5, 'o');
#    plt.plot(0.5, GRIDSIZE[0] - 0.5, 'x', color='r');
#    plt.plot(GRIDSIZE[1] - 0.5, 0.5, 'x', color='r');
    #plot(maze, GRIDSIZE);
    if maze[0][0] == 1:
        return 0, [], maze, 0;
    fringe = [ (abs( (GRIDSIZE[0]-1) + (GRIDSIZE[1]-1) )  , (0, 0), 0.0, (-1, -1)) ];
    goal = (GRIDSIZE[0]-1, GRIDSIZE[1]-1);
    path = [ [(-1, -1) for j in xrange(GRIDSIZE[0])] for i in xrange(GRIDSIZE[1])]; #will be filled later
    maze[0][0] = -1;
    explored = 0;
    current_size = 1; max_size = 1;
    while len(fringe) != 0:
        current = heapq.heappop(fringe);
        current_size -= 1;
        explored += 1;
        path[current[1][0]][current[1][1]] = current[-1];
#        plt.plot(current[1][1] + 0.5, GRIDSIZE[0] - current[1][0] - 0.5, 'x', color='blue');
        if current[1] == goal:
            break;
        else:
            for child in return_neighbors(current[1], maze, GRIDSIZE):
                current_size += 1;
                heuristic = abs( (child[0]-(GRIDSIZE[0]-1)) + ((child[1]-(GRIDSIZE[1]-1))) )
                actual_cost = current[2] + 1;
                maze[child[0]][child[1]] = -1;
                heapq.heappush(fringe, (heuristic + actual_cost, child, actual_cost, current[1]));
            if current_size > max_size:
                max_size = current_size;
    ##backtrack the path here starting from endpoint
    if current[1] != goal:
        return explored, [], maze, max_size;
    current_node = goal;
    shortest_path = [];
    while current_node != (-1, -1):
        maze[current_node[0]][current_node[1]] = 2;
        shortest_path.append(current_node);
#        plt.plot(current_node[1] + 0.5, GRIDSIZE[0] - current_node[0] - 0.5, 'o', color='red');
        current_node = path[current_node[0]][current_node[1]];
#    plt.show();
    return explored, shortest_path, maze, max_size;

def plot_v2(maze, name, path, visited):
    #print path[0];
    directions = {'Down': (1, 0),
            'Up': (-1, 0),
            'Right': (0, 1),
            'Left': (0, -1)};
    #print directions;
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
     #   print 'cuurent dir: ', dir;
        plt.plot( start[1] + 0.5, GRIDSIZE[0] - start[0] - 0.5, 'o', color='red');
        start[0] += directions[dir][0]; start[1] += directions[dir][1]; 
    plt.plot( start[1] + 0.5, GRIDSIZE[0] - start[0] - 0.5, 'o', color='red');
    plt.savefig(name);
    plt.gcf().clear();

# graph represented as dict; keys = node coord, values = neighbor coord (tuple with 2 num)
def convertMaze(maze):
    height = len(maze)
    width = len(maze[0]) if height else 0
    graph = {(i, j): [] for j in range(width) for i in range(height) if not maze[i][j]}
#    print graph
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
    if maze[0][0] == 1:
        return 0, [], [], 0;
    start, goal = (0, 0), (len(maze) - 1, len(maze[0]) - 1)
    queue = deque([([], start)])
    visited = set()
    graph = convertMaze(maze)
    explored = 0;
    current_size = 1; max_size = 1;
    while queue:
        path, current = queue.popleft()
        current_size -= 1;
        if current == goal:
            return explored, path, visited, max_size
        if current in visited:
            continue
        visited.add(current)
        explored += 1;
        for direction, neighbour in graph[current]:
            queue.append((path + [direction], neighbour))
            current_size += 1;
        if current_size > max_size:
            max_size = current_size;
    return explored, [], visited, max_size;


# bfs with stack
def solveDFS(maze):
    if maze[0][0] == 1:
        return 0, [], [], 0;
    start, goal = (0, 0), (len(maze) - 1, len(maze[0]) - 1)
    stack = deque([([], start)])
    visited = set()
    graph = convertMaze(maze)
    explored = 0;
    current_size = 1; max_size = 1;
    while stack:
        path, current = stack.pop()
        current_size -= 1;
        if current == goal:
            return explored, path, visited, max_size;
        if current in visited:
            continue
        visited.add(current)
        explored += 1
        for direction, neighbour in graph[current]:
            current_size += 1;
            stack.append((path + [direction], neighbour))
        if current_size > max_size:
            max_size = current_size
    return explored, [], visited, max_size;

def copy_maze(maze, GRIDSIZE):
    new_maze = [[maze[i][j] for j in xrange(GRIDSIZE[1])]for i in xrange(GRIDSIZE[0])];
    return new_maze;
#    for i in xrange(
#if __name__ == "__main__":
#    GRIDSIZE = (20, 20);
#    for p in [0.3]:
#        maze = generate_maze(p, GRIDSIZE);
##        operate_maze = maze[:][:];
#        time_1 = time.time();
#        cost, path, solved_maze = astar_man(copy_maze(maze, GRIDSIZE), GRIDSIZE);
#        time_2 = time.time();
#        plot(solved_maze, GRIDSIZE, 'astar_man.png');
#        print 'astar_man solved', len(path), time_2-time_1, 'probability: %0.1f' % p;
#        time_1 = time.time();
#        cost, path, solved_maze_2 = astar(copy_maze(maze, GRIDSIZE), GRIDSIZE);
#        time_2 = time.time();
#        print 'astar_euc solved', len(path), time_2-time_1, 'probability: %0.1f' % p;
#        plot(solved_maze_2, GRIDSIZE, 'astar_euc.png');
#        time_1 = time.time();
#        explored_number, path, visited = solveBFS(maze);
#        time_2 = time.time()
#        print 'bfs solved', len(path), time_2 - time_1, 'probability: %0.1f' % p;
#        plot_v2(maze, 'bfs.png', path, visited);
#        time_1 = time.time();
#        explored_number, path, visited = solveDFS(maze);
#        time_2 = time.time();
#        print 'dfs solved', len(path), time_2 - time_1, 'probability: %0.1f' % p;
#        plot_v2(maze, 'dfs.png', path, visited);
