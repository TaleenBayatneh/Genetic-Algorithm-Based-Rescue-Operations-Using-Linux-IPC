#include "astar.h"
#include "utils.h"
#include "map.h"
#include "discovery.h"


//initialize the priority queue by setting its size to zero
static void pq_init(PriorityQueue* pq) {
    pq->size = 0;
}

//insert a new node into the priority queue
static void pq_push(PriorityQueue* pq, AStarNode* node) {
    if (pq->size >= MAX_PATH_LENGTH * 10 - 1) return;
    
    int i = pq->size++;
    //insert node at the end of the heap
    pq->nodes[i] = node;
    
    //restore heap property by bubbling the node upward
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (pq->nodes[i]->f_cost < pq->nodes[parent]->f_cost) {
            AStarNode* tmp = pq->nodes[i];
            pq->nodes[i] = pq->nodes[parent];
            pq->nodes[parent] = tmp;
            i = parent;
        } else {
            break;
        }
    }
}

//remove and return the node with the lowest f_cost
static AStarNode* pq_pop(PriorityQueue* pq) {
    if (pq->size == 0) return NULL;
    
    //save the root node (minimum f_cost)
    AStarNode* result = pq->nodes[0];
    //move last node to root position
    pq->nodes[0] = pq->nodes[--pq->size];
    
    //restore heap property by bubbling downward
    int i = 0;
    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;
        
        if (left < pq->size && pq->nodes[left]->f_cost < pq->nodes[smallest]->f_cost) {
            smallest = left;
        }
        if (right < pq->size && pq->nodes[right]->f_cost < pq->nodes[smallest]->f_cost) {
            smallest = right;
        }
        
        if (smallest != i) {
            AStarNode* tmp = pq->nodes[i];
            pq->nodes[i] = pq->nodes[smallest];
            pq->nodes[smallest] = tmp;
            i = smallest;
        } else {
            break;
        }
    }
    
    return result;
}

//check if the priority queue is empty
static int pq_is_empty(PriorityQueue* pq) {
    return pq->size == 0;
}



int astar_find_path(const SharedData* shared, Point3D start, Point3D goal,
                    Chromosome* result_path, const Config* config) {

    (void)config;
    if (!shared || !result_path) return 0;
    
    //check if start and goal are accessible
    if (!map_is_accessible(shared, start.x, start.y, start.z) ||
        !map_is_accessible(shared, goal.x, goal.y, goal.z)) {
        return 0;
    }
    
    //priority queue holding nodes to be explored (min f_cost first)
    PriorityQueue open_set;
    pq_init(&open_set);
    
    //marks visited cells to avoid reprocessing them
    int closed[MAX_X][MAX_Y][MAX_Z];
    memset(closed, 0, sizeof(closed));
    
    //maps each grid cell to its corresponding A* node
    AStarNode* node_map[MAX_X][MAX_Y][MAX_Z];
    memset(node_map, 0, sizeof(node_map));
    
    //preallocated memory pool for A* nodes
    AStarNode* node_pool = malloc(sizeof(AStarNode) * MAX_PATH_LENGTH * 10);
    if (!node_pool) return 0;
    int node_count = 0;
    
    //create start node
    AStarNode* start_node = &node_pool[node_count++];
    start_node->pos = start;//start position
    start_node->g_cost = 0;
    start_node->h_cost = map_euclidean_distance(start, goal);//Heuristic
    start_node->f_cost = start_node->g_cost + start_node->h_cost;
    start_node->parent = NULL;
    start_node->direction = -1;
    
    //register start node in node map
    node_map[start.x][start.y][start.z] = start_node;
    //push start node into open set
    pq_push(&open_set, start_node);
    
    //MAIN A* LOOP
    AStarNode* goal_node = NULL;
    while (!pq_is_empty(&open_set) && node_count < MAX_PATH_LENGTH * 10 - 30) {
        //get node with lowest f_cost
        AStarNode* current = pq_pop(&open_set);
        //if goal reached so stop searching
        if (point3d_equals(current->pos, goal)) {
            goal_node = current;
            break;
        }
        //skip node if already visited
        if (closed[current->pos.x][current->pos.y][current->pos.z]) {
            continue;
        }
        //mark current cell as visited
        closed[current->pos.x][current->pos.y][current->pos.z] = 1;
        

        //Explore neighbors
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            //compute neighboring cell position
            Point3D next = utils_apply_direction(current->pos, d);
            //skip if cell is not accessible
            if (!map_is_accessible(shared, next.x, next.y, next.z)) continue;
            //skip if neighbor already visited
            if (closed[next.x][next.y][next.z]) continue;
            
            //base movement cost:
            //straight move = 1, diagonal move = sqrt(2)
            double move_cost = (d < 8) ? 1.0 : 1.414;
            //add risk-based cost
            move_cost += map_get_risk(shared, next.x, next.y, next.z) / 100.0;
            //total cost from start through current node
            double new_g = current->g_cost + move_cost;
            
            //retrieve or create neighbor node
            AStarNode* neighbor = node_map[next.x][next.y][next.z];
            
            if (!neighbor) {
                //first time visiting this cell → create node
                neighbor = &node_pool[node_count++];
                neighbor->pos = next;
                neighbor->g_cost = DBL_MAX;
                neighbor->parent = NULL;
                node_map[next.x][next.y][next.z] = neighbor;
            }
            
            //check if this path is better than previous one
            if (new_g < neighbor->g_cost) {
                neighbor->g_cost = new_g;//update costs
                neighbor->h_cost = map_euclidean_distance(next, goal);
                neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
                //store parent and movement direction
                neighbor->parent = current;
                neighbor->direction = d;
                //push updated node to open set
                pq_push(&open_set, neighbor);
            }
        }
    }
    //if goal was never reached
    if (!goal_node) {
        free(node_pool);
        return 0;
    }
    
    //reconstruct path
    int path_len = 0;
    AStarNode* node = goal_node;

    //count path length by following parents
    while (node) {
        path_len++;
        node = node->parent;
    }

    //path too long so invalid
    if (path_len > MAX_PATH_LENGTH) {
        free(node_pool);
        return 0;
    }
    
    //initialize result chromosome
    memset(result_path, 0, sizeof(Chromosome));
    result_path->length = path_len;
    result_path->is_valid = 1;
    result_path->start_pos = start;
    result_path->end_pos = goal;
    
    //fill path in reverse order (start to goal)
    node = goal_node;
    for (int i = path_len - 1; i >= 0; i--) {
        result_path->genes[i].position = node->pos;
        result_path->genes[i].direction = node->direction;
        node = node->parent;
    }
    
    //Calculate path length
    result_path->path_length = 0;
    for (int i = 1; i < result_path->length; i++) {
        result_path->path_length += map_euclidean_distance(
            result_path->genes[i-1].position,
            result_path->genes[i].position
        );
    }
    
    free(node_pool);
    //return number of nodes in the final path
    return path_len;
}



int astar_find_path_discovered(const SharedData* shared, Point3D start, Point3D goal,
                               Chromosome* result_path, const Config* config) {
    (void)config;
    if (!shared || !result_path) return 0;
    
    //Check if start and goal are discovered and accessible
    if (shared->discovered[start.x][start.y][start.z] == CELL_UNKNOWN ||
        shared->discovered[start.x][start.y][start.z] == CELL_DEBRIS) {
        return 0;
    }
    if (shared->discovered[goal.x][goal.y][goal.z] == CELL_UNKNOWN ||
        shared->discovered[goal.x][goal.y][goal.z] == CELL_DEBRIS) {
        return 0;
    }
    
    //priority queue for A* (nodes ordered by f_cost)
    PriorityQueue open_set;
    pq_init(&open_set);
    
    //tracks visited cells
    int closed[MAX_X][MAX_Y][MAX_Z];
    memset(closed, 0, sizeof(closed));
    
    //one node per cell (avoid duplicates)
    AStarNode* node_map[MAX_X][MAX_Y][MAX_Z];
    memset(node_map, 0, sizeof(node_map));
    
    // Preallocated nodes
    AStarNode* node_pool = malloc(sizeof(AStarNode) * MAX_PATH_LENGTH * 10);
    if (!node_pool) return 0;
    int node_count = 0;
    
    //create the start node
    AStarNode* start_node = &node_pool[node_count++];
    start_node->pos = start;
    start_node->g_cost = 0;
    start_node->h_cost = map_euclidean_distance(start, goal);
    start_node->f_cost = start_node->g_cost + start_node->h_cost;
    start_node->parent = NULL;
    start_node->direction = -1;
    
    //register start node
    node_map[start.x][start.y][start.z] = start_node;
    pq_push(&open_set, start_node);
    
    AStarNode* goal_node = NULL;

    //Main A* search loop
    while (!pq_is_empty(&open_set) && node_count < MAX_PATH_LENGTH * 10 - 30) {
        AStarNode* current = pq_pop(&open_set);
        
        //if goal reached so stop search
        if (point3d_equals(current->pos, goal)) {
            goal_node = current;
            break;
        }
        
        //skip if this cell was already processed
        if (closed[current->pos.x][current->pos.y][current->pos.z]) {
            continue;
        }
        //mark current cell as visited
        closed[current->pos.x][current->pos.y][current->pos.z] = 1;
        
        //explore all possible movement directions
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            //compute neighbor position
            Point3D next = utils_apply_direction(current->pos, d);
            
            //ensure neighbor is inside map
            if (next.x < 0 || next.x >= shared->dim_x ||
                next.y < 0 || next.y >= shared->dim_y ||
                next.z < 0 || next.z >= shared->dim_z) {
                continue;
            }
            
            //only allow movement in discovered cells
            int cell = shared->discovered[next.x][next.y][next.z];
            if (cell == CELL_UNKNOWN || cell == CELL_DEBRIS) continue;

            //skip already visited cells
            if (closed[next.x][next.y][next.z]) continue;
            
            //movement cost
            double move_cost = (d < 8) ? 1.0 : 1.414;
            double new_g = current->g_cost + move_cost;
            
            //retrieve or create neighbor node
            AStarNode* neighbor = node_map[next.x][next.y][next.z];
            
            if (!neighbor) {
                neighbor = &node_pool[node_count++];
                neighbor->pos = next;
                neighbor->g_cost = DBL_MAX;//initialize with large value
                neighbor->parent = NULL;
                node_map[next.x][next.y][next.z] = neighbor;
            }
            
            //update node if a cheaper path is found
            if (new_g < neighbor->g_cost) {
                neighbor->g_cost = new_g;
                neighbor->h_cost = map_euclidean_distance(next, goal);
                neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
                neighbor->parent = current;
                neighbor->direction = d;
                //push updated node into open set
                pq_push(&open_set, neighbor);
            }
        }
    }
    
    //if goal was not reached so fail
    if (!goal_node) {
        free(node_pool);
        return 0;
    }
    //count path length by following parent pointers
    int path_len = 0;
    AStarNode* node = goal_node;
    while (node) {
        path_len++;
        node = node->parent;
    }
    
    //path too long so invalid
    if (path_len > MAX_PATH_LENGTH) {
        free(node_pool);
        return 0;
    }

    //initialize output chromosome
    memset(result_path, 0, sizeof(Chromosome));
    result_path->length = path_len;
    result_path->is_valid = 1;
    result_path->start_pos = start;
    result_path->end_pos = goal;
    
    //fill path from end to start
    node = goal_node;
    for (int i = path_len - 1; i >= 0; i--) {
        result_path->genes[i].position = node->pos;
        result_path->genes[i].direction = node->direction;
        node = node->parent;
    }
    
    //compute total path length
    result_path->path_length = 0;
    for (int i = 1; i < result_path->length; i++) {
        result_path->path_length += map_euclidean_distance(
            result_path->genes[i-1].position,
            result_path->genes[i].position
        );
    }
    
    free(node_pool);
    //return number of steps in the path
    return path_len;
}



int astar_find_nearest_survivor(const SharedData* shared, Point3D start,
                                Chromosome* result_path, int* survivor_id,
                                const Config* config) {
    double min_dist = DBL_MAX;//initialize minimum distance with a very large value
    int nearest = -1;//index of the nearest survivor (-1 means none found yet)
    
    //loop over all survivors in the map
    for (int s = 0; s < shared->num_survivors; s++) {
        //consider only survivors that are not rescued yet
        if (!shared->survivors[s].is_rescued) {
            //compute Euclidean distance from robot start to survivor
            double dist = map_euclidean_distance(start, shared->survivors[s].position);
            //update nearest survivor if this one is closer
            if (dist < min_dist) {
                min_dist = dist;
                nearest = s;
            }
        }
    }
    
    //if no unrescued survivor was found return failure
    if (nearest < 0) return 0;
    
    //output the ID of the nearest survivor
    *survivor_id = nearest;

    //run A* path planning to the nearest survivor
    return astar_find_path(shared, start, shared->survivors[nearest].position,
                          result_path, config);
}


AStarResults astar_run_full_knowledge(SharedData* shared, const Config* config) {
    AStarResults results = {0, 0, 0, 0, 0};//structure to store A* performance results
    
    //log that we are running baseline A* with full map knowledge
    utils_log(LOG_INFO, "Running A* with full map knowledge (baseline)...");
    
    //backup current rescued state of survivors
    int rescued_backup[MAX_SURVIVORS];
    for (int s = 0; s < shared->num_survivors; s++) {
        rescued_backup[s] = shared->survivors[s].is_rescued;
        shared->survivors[s].is_rescued = 0; //reset rescued flag for fair A* simulation
    }
    
    //choose a random entry point as starting position
    Point3D current = map_get_random_entry(shared);
    double total_time = 0;//total sequential time for A* robot
    
    //print header for A* execution
    printf("\n%sA* Single Robot (Full Knowledge):%s\n", COLOR_CYAN, COLOR_RESET);
    printf("  Starting at (%d,%d,%d)\n", current.x, current.y, current.z);
    
    //main loop: rescue survivors one by one
    for (int i = 0; i < shared->num_survivors; i++) {
        Chromosome path;
        int survivor_id;
        
        //find nearest survivor and plan path using A*
        if (astar_find_nearest_survivor(shared, current, &path, &survivor_id, config) > 0) {
            //accumulate total path length
            results.total_path_length += path.path_length;
            results.paths_found++;
            
            //compute time for this path
            double path_time = path.path_length / config->robot_speed + config->rescue_time;
            total_time += path_time;
            
            //print detailed info for this rescue
            printf("  -> Survivor %d at (%d,%d,%d): path=%.2f, time=%.2f\n",
                   survivor_id,
                   shared->survivors[survivor_id].position.x,
                   shared->survivors[survivor_id].position.y,
                   shared->survivors[survivor_id].position.z,
                   path.path_length, path_time);
            
            //update robot position to survivor location
            current = path.end_pos;
            //mark survivor as rescued
            shared->survivors[survivor_id].is_rescued = 1;
            results.survivors_rescued++;
        } else {
            //no path could be found to remaining survivors
            printf("  -> Could not find path to remaining survivors\n");
            break;
        }
    }
    
    //store total execution time
    results.total_time = total_time;
    //compute average path length
    if (results.paths_found > 0) {
        results.avg_path_length = results.total_path_length / results.paths_found;
    }
    
    //restore original rescued state of survivors
    for (int s = 0; s < shared->num_survivors; s++) {
        shared->survivors[s].is_rescued = rescued_backup[s];
    }
    
    return results;
}



void astar_compare_with_online_ga(SharedData* shared, const AStarResults* astar_results,
                                  double ga_total_time, const Config* config) {
    (void)config;
    
    //Calculate GA stats
    double ga_total_path = 0;//accumulate total path length traveled by GA robots
    double ga_parallel_time = 0;//GA execution time assuming parallel robots (max robot time)
    int ga_rescued = shared->survivors_rescued; //number of survivors rescued by GA
    
    //collect GA statistics from all robots
    for (int r = 0; r < shared->num_robots; r++) {
        Robot* robot = &shared->robots[r];
        ga_total_path += robot->steps_taken;
        if (robot->total_time > ga_parallel_time) {
            ga_parallel_time = robot->total_time;
        }
    }
    
    printf(" \n                   GA vs A* COMPARISON                          \n");
    
    printf("\n┌─────────────────────────┬─────────────────┬─────────────────┐\n");
    printf("│ Metric                  │      GA         │ A* (Full Know.) │\n");
    printf("├─────────────────────────┼─────────────────┼─────────────────┤\n");
    printf("│ Survivors Rescued       │ %15d │ %15d │\n", 
           ga_rescued, astar_results->survivors_rescued);
    printf("│ Total Path Length       │ %15.2f │ %15.2f │\n", 
           ga_total_path, astar_results->total_path_length);
    printf("│ Sequential Time (sec)   │ %15.2f │ %15.2f │\n", 
           ga_total_time, astar_results->total_time);
    printf("│ Parallel Time (sec)     │ %15.2f │ %15s │\n", 
           ga_parallel_time, "N/A");
    printf("│ Robots Used             │ %15d │ %15d │\n", 
           shared->num_robots, 1);
    printf("│ Map Knowledge           │ %14.1f%% │ %14.1f%% │\n", 
           discovery_get_progress(shared), 100.0);
    printf("│ GA Replans              │ %15d │ %15s │\n", 
           shared->stats.total_replans, "N/A");
    printf("└─────────────────────────┴─────────────────┴─────────────────┘\n");
    
    
    printf("\n%s=== Analysis ===%s\n", COLOR_BOLD, COLOR_RESET);
    
    if (ga_rescued >= astar_results->survivors_rescued) {
        printf("%s  GA rescued same or more survivors%s\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s A* rescued more survivors (had full map knowledge)%s\n", COLOR_YELLOW, COLOR_RESET);
    }
    
    if (ga_parallel_time < astar_results->total_time && ga_rescued > 0) {
        double speedup = astar_results->total_time / ga_parallel_time;
        printf("%s  GA parallel speedup: %.2fx faster%s\n", COLOR_GREEN, COLOR_RESET, speedup);
    }
    
    double exploration = discovery_get_progress(shared);
    if (exploration < 100.0 && ga_rescued == shared->num_survivors) {
        printf("%s  GA rescued all survivors with only %.1f%% exploration%s\n", 
               COLOR_GREEN, COLOR_RESET, exploration);
    }
    
    
    double ga_efficiency = (ga_rescued > 0) ? ga_total_path / ga_rescued : 0;
    double astar_efficiency = (astar_results->survivors_rescued > 0) ? 
                              astar_results->total_path_length / astar_results->survivors_rescued : 0;
    
    printf("\nPath Efficiency (lower is better):\n");
    printf("  GA: %.2f steps per survivor\n", ga_efficiency);
    printf("  A*:        %.2f steps per survivor\n", astar_efficiency);
    
    if (ga_efficiency > 0 && astar_efficiency > 0) {
        double overhead = ((ga_efficiency - astar_efficiency) / astar_efficiency) * 100;
        if (overhead > 0) {
            printf("  GA overhead: %.1f%% (due to exploration)\n", overhead);
        } else {
            printf("   GA was %.1f%% more efficient!\n", -overhead);
        }
    }
}


void astar_print_results(const AStarResults* results) {
    printf("\n%sA* Results Summary:%s\n", COLOR_CYAN, COLOR_RESET);
    printf("  Survivors Rescued: %d\n", results->survivors_rescued);
    printf("  Total Path Length: %.2f\n", results->total_path_length);
    printf("  Total Time:        %.2f seconds\n", results->total_time);
    printf("  Paths Found:       %d\n", results->paths_found);
    printf("  Avg Path Length:   %.2f\n", results->avg_path_length);
}