import random
from maze_runner import astar_man
from maze_runner import astar
from maze_runner import plot
from maze_runner import generate_maze
from maze_runner import copy_maze
from maze_runner import solveBFS
from maze_runner import plot_v2
from maze_runner import solveDFS


def single_crossover(maze_1, maze_2, num_rows):
    position = random.randint(1, num_rows);
    child_1 = [list(x) for x in maze_1[:position]] + [list(x) for x in maze_2[position:]];
    child_1[GRIDSIZE[0]-1][GRIDSIZE[1]-1] = 0;
    child_2 = [list(x) for x in maze_1[position:]] + [list(x) for x in maze_2[:position]];
    child_2[0][0] = 0;
    return child_1, child_2;

def mutate_children(child, GRIDSIZE):
    #print child, GRIDSIZE
    total_cells = GRIDSIZE[0] * GRIDSIZE[1];
    percentage = abs(random.random() - 0.5);
    for i in xrange(int(percentage * total_cells)):
        row = random.randint(0, GRIDSIZE[0]-1);
        col = random.randint(0, GRIDSIZE[1]-1);
        #print row, col;
        if (row, col) == (0,0) or (row, col) == (GRIDSIZE[0]-1, GRIDSIZE[1]-1):
            continue;
        child[row][col] = 0 if child[row][col] == 1 else 1;

    return child;

def find_hard_maze(maze_1, maze_2, GRIDSIZE, cost_1, cost_2, cost_type, algo_type, run):
    ##cost_type can have three values
    #0 is for explored_nodes
    #1 is for path length
    #2 is for the maximum size of fringe
    mutation_probability = 0.5;
    max_tries = 1000; cur_max_cost = max(cost_1, cost_2);
    cost_1 = -1; cost_2 = -1; cur_max_child = [[]];
    all_costs = [0,0,0]; cur_tries = 0; child_1 = [[]]; child_2 = [[]];
    flag = True;
    while True:
        #print 'searching';
        while flag or (max(cost_1, cost_2) < cur_max_cost and cur_tries < max_tries):
            #print 'searching'
            flag = False;
            #print 'searching';
            child_1, child_2 = single_crossover(maze_1, maze_2, GRIDSIZE[0]);
            #print child_1;
            #child_1[0][0] = 1;
            #print maze_1[0][0];
            #break;
            
            if random.random() > 1 - mutation_probability:
                child_1 = mutate_children(child_1, GRIDSIZE);
            if random.random() > 1 - mutation_probability:
                child_2 = mutate_children(child_2, GRIDSIZE);

            if algo_type == 0:
                all_costs[0], all_costs[1], _, all_costs[2] = astar(copy_maze(child_1, GRIDSIZE), \
                    GRIDSIZE);
            elif algo_type == 1:
                all_costs[0], all_costs[1], _, all_costs[2] = astar_man(copy_maze(child_1, GRIDSIZE), \
                        GRIDSIZE);
            elif algo_type == 2:
                all_costs[0], all_costs[1], _, all_costs[2] = solveBFS(child_1);
            elif algo_type == 3:
                all_costs[0], all_costs[1], _, all_costs[2] = solveDFS(child_1);

            all_costs[1] = len(all_costs[1]);
            
            if all_costs[1] == 0:
                flag = True;
            cost_1 = all_costs[cost_type];
            
            if algo_type == 0:
                all_costs[0], all_costs[1], _, all_costs[2] = astar(copy_maze(child_2, GRIDSIZE), \
                        GRIDSIZE);
            elif algo_type == 1:    
                all_costs[0], all_costs[1], _, all_costs[2] = astar_man(copy_maze(child_2, GRIDSIZE), \
                        GRIDSIZE);
            elif algo_type == 2:
                all_costs[0], all_costs[1], _, all_costs[2] = solveBFS(child_2);
            elif algo_type == 3:
                all_costs[0], all_costs[1], _, all_costs[2] = solveDFS(child_2);


            all_costs[1] = len(all_costs[1]);

            if all_costs[1] == 0:
                flag = True;
            cost_2 = all_costs[cost_type];
            cur_tries += 1;

        if cur_tries >= max_tries:
            break;
        else:
            cur_tries = 0;
            maze_1 = child_1;
            maze_2 = child_2;
            cur_max_cost = max(cost_1, cost_2);
            cur_max_child = child_1 if cost_1 > cost_2 else child_2;
            cost_1 = -1; cost_2 = -1;

    if algo_type == 0:
        _, _, solved_child_1, _ = astar(cur_max_child, GRIDSIZE);
    elif algo_type == 1:
        _, _, solved_child_1, _ = astar_man(cur_max_child, GRIDSIZE);
    elif algo_type == 2:
        _, path_1, visited_1, _ = solveBFS(cur_max_child);
    elif algo_type == 3:    
        _, path_1, visited_1, _ = solveDFS(cur_max_child);
    
    if algo_type <= 1:
        plot(solved_child_1, GRIDSIZE, 'max_child_'+ str(algo_type) + '_' + str(cost_type) + \
            '_' + str(run) + '.png');
    else:
        plot_v2(cur_max_child, 'max_child_'+ str(algo_type) + '_' + str(cost_type) + \
            '_' + str(run) + '.png', path_1, visited_1);
    #plot(solved_child_2, GRIDSIZE, 'child_2.png');
    print 'Max Cost Child: ', cur_max_cost, ' for algo type: ', algo_type, ' and cost type: ', cost_type, \
        'for run: ', run;
    ####bfs and dfs plotting and testing code
    ##bfs
    #_, path_1, visited_1, _ = solveBFS(child_1);
    #_, path_2, visited_2, _ = solveBFS(child_2);
    #plot_v2(child_1, 'child_1.png', path_1, visited_1);
    #plot_v2(child_2, 'child_2.png', path_2, visited_2);
    
    ##bfs and dfs testing goes down here
    
    #print 'i got two mazes';
    return None;

if __name__ == '__main__':
    p = 0.2; path = []; maze_1 = [[]]; maze_2 = [[]]; len_1 = -1; len_2 = -1;
    GRIDSIZE = (50,50);
    all_costs_1 = [0,0,0]; all_costs_2 = [0,0,0];
    ##change this and everything changes
    cost_type = 1;
    solved_1 = [[]]; solved_2 = [[]]; visited_1 = [[]]; visited_2 = [[]];
    ##find two solvable mazes below
    while path == []:
        maze_1 = generate_maze(0.2, GRIDSIZE);
        all_costs_1[0], path, solved_1, all_costs_1[2] = astar(copy_maze(maze_1, GRIDSIZE), GRIDSIZE);
    all_costs_1[1]  = len(path);
    
    path = [];
    while path == []:
        maze_2 = generate_maze(0.2, GRIDSIZE);
        all_costs_2[0], path, solved_2, all_costs_2[2] = astar(copy_maze(maze_2, GRIDSIZE), GRIDSIZE);
    all_costs_2[1] = len(path);
    
    ##we have found the maze, now make it do everything
    ##0 is for astar euclidean
    ##1 is for astar manhattan
    ##2 is for bfs
    ##3 is for dfs
    for algo_type in xrange(4):
        if algo_type == 0:
            all_costs_1[0], all_costs_1[1], solved_1, all_costs_1[2] = astar(copy_maze(maze_1, GRIDSIZE), GRIDSIZE);
            all_costs_2[0], all_costs_2[1], solved_2, all_costs_2[2] = astar(copy_maze(maze_2, GRIDSIZE), GRIDSIZE);
        elif algo_type == 1:
            all_costs_1[0], all_costs_1[1], solved_1, all_costs_1[2] = astar_man(copy_maze(maze_1, GRIDSIZE), GRIDSIZE);
            all_costs_2[0], all_costs_2[1], solved_2, all_costs_2[2] = astar_man(copy_maze(maze_2, GRIDSIZE), GRIDSIZE);
        elif algo_type == 2:
            all_costs_1[0], all_costs_1[1], visited_1, all_costs_1[2] = solveBFS(maze_1);
            all_costs_2[0], all_costs_2[1], visited_2, all_costs_2[2] = solveBFS(maze_2);
        elif algo_type == 3:
            all_costs_1[0], all_costs_1[1], visited_1, all_costs_1[2] = solveDFS(maze_1);
            all_costs_2[0], all_costs_2[1], visited_2, all_costs_2[2] = solveDFS(maze_2);
        
        if algo_type <= 1:
            plot(solved_1, GRIDSIZE, 'parent1_' + str(algo_type) + '.png');
            plot(solved_2, GRIDSIZE, 'parent2_' + str(algo_type) + '.png');
        else:
            plot_v2(maze_1, 'parent1_' + str(algo_type) + '.png', all_costs_1[1], visited_1);
            plot_v2(maze_2, 'parent2_' + str(algo_type) + '.png', all_costs_2[1], visited_2);
        
        all_costs_1[1] = len(all_costs_1[1]);
        all_costs_2[1] = len(all_costs_2[1]);
        for cost_type in xrange(3):
            for run in xrange(3):
                print 'Parent\'s Max Cost: ', max(all_costs_1[cost_type], all_costs_2[cost_type]), ' for algo type: ', algo_type, \
                        ' cost type: ', cost_type;
                find_hard_maze(maze_1, maze_2, GRIDSIZE, all_costs_1[cost_type], all_costs_2[cost_type], cost_type, algo_type, run);
    
