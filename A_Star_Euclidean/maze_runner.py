import random
import heapq
from math import sqrt
import matplotlib.pyplot as plt

GRIDSIZE = (10,10);
#maze = [[1 if random.random() > 0.7 else 0 \
        #for x in xrange(GRIDSIZE[0])] for i in xrange(GRIDSIZE[1])];
maze = [[0, 1, 0, 1, 0, 0, 0, 0, 0, 0],

        [0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
        
        [0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
        
        [1, 0, 1, 0, 0, 0 ,0, 0 ,0, 0],
        
        [0, 0, 1, 0, 0, 0, 1, 0, 1, 1],
        
        [1, 0, 0, 1, 0, 0, 0, 1, 0, 0],
        
        [0, 0, 1, 0, 0, 0, 1, 1, 0, 1],
        
        [0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
        
        [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
        
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]];

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
                
# return types is (num_explored, path_itself, modified_maze_for_plotting);
def astar(maze, GRIDSIZE):
#    for i in xrange(GRIDSIZE[0]):
#        for j in xrange(GRIDSIZE[1]):
#            if maze[i][j] == 1:
#                plt.plot( j + 0.5, GRIDSIZE[0] - i - 0.5, 'o');
#    plt.plot(0.5, GRIDSIZE[0] - 0.5, 'x', color='r');
#    plt.plot(GRIDSIZE[1] - 0.5, 0.5, 'x', color='r');
    plot(maze, GRIDSIZE);
    fringe = [ (sqrt( ( float(GRIDSIZE[0]-1)**2 + float(GRIDSIZE[1]-1)**2) ), (0, 0), 0.0, (-1, -1)) ];
    goal = (GRIDSIZE[0]-1, GRIDSIZE[1]-1);
    path = [ [(-1, -1) for j in xrange(GRIDSIZE[0])] for i in xrange(GRIDSIZE[1])]; #will be filled later
    maze[0][0] = -1;
    explored = 0;
    while len(fringe) != 0:
        current = heapq.heappop(fringe);
        explored += 1;
        path[current[1][0]][current[1][1]] = current[-1];
#        plt.plot(current[1][1] + 0.5, GRIDSIZE[0] - current[1][0] - 0.5, 'x', color='blue');
        if current[1] == goal:
            break;
        else:
            for child in return_neighbors(current[1], maze, GRIDSIZE):
                heuristic = sqrt(((child[0]-(GRIDSIZE[0]-1))**2) + ((child[1]-(GRIDSIZE[1]-1))**2))
                actual_cost = current[2] + 1;
                maze[child[0]][child[1]] = -1;
                heapq.heappush(fringe, (heuristic + actual_cost, child, actual_cost, current[1]));
    ##backtrack the path here starting from endpoint
    if current[1] != goal:
        return float('inf'), [], maze;
    current_node = goal;
    shortest_path = [];
    while current_node != (-1, -1):
        maze[current_node[0]][current_node[1]] = 2;
        shortest_path.append(current_node);
#        plt.plot(current_node[1] + 0.5, GRIDSIZE[0] - current_node[0] - 0.5, 'o', color='red');
        current_node = path[current_node[0]][current_node[1]];
#    plt.show();
    return explored, shortest_path, maze;

if __name__ == "__main__":
    cost, path, solved_maze = astar(maze, GRIDSIZE);
    plot(solved_maze, GRIDSIZE);
    print "Cost: ", cost;
    print "Path: ", path;
