//robot modulemanages robot lifecycle, state machine, sensing, planning, moving, rescuing, and ipc updates

#include "robot.h"        //robot api + shared structs
#include "online_ga.h"    //online planning using ga (chromosome paths)
#include "discovery.h"    //discovery scan + survivor detection + knowledge sync
#include "utils.h"        //logging, random seed, time helpers
#include "map.h"          //map helpers (entry points, distance, etc.)

//initialize all robots and place each one at a random entry point
void robot_init_all(SharedData* shared, const Config* config) {
    utils_log(LOG_INFO,
              "Initializing %d robots for online operation",
              config->num_robots);                                        //log how many robots we will spawn
    
    for (int i = 0; i < config->num_robots; i++) {                        //loop robots
        Point3D entry = map_get_random_entry(shared);                     //pick a spawn/entry point
        robot_init(&shared->robots[i], i, entry, config);                 //setup robot struct fields
    }
}

                                                                          //initialize a single robot struct with default values and empty local knowledge
void robot_init(Robot* robot, int id, Point3D start_pos, const Config* config) {
    (void)config;                                                         //config not needed for init fields here
    memset(robot, 0, sizeof(Robot));                                      //clear all fields to known defaults
    
    robot->id = id;                                                       //unique robot id
    robot->pid = 0;                                                       //process id filled later when process starts
    robot->current_pos = start_pos;                                       //current location (spawn)
    robot->start_pos = start_pos;                                         //original spawn position
    robot->base_pos = start_pos;                                          //base station is same as entry point
    
    robot->has_supplies = 1;                                              //robot starts with supplies loaded
    robot->current_survivor_id = -1;                                      //no target survivor at start
    
    robot->is_active = 1;                                                 //robot considered active after init
    robot->is_finished = 0;                                               //not finished yet
    
                                                                              //rescue tracking arrays so we can report which survivors each robot saved
    robot->num_survivors_saved = 0;                                       //nothing saved yet
    robot->total_trips = 0;                                               //trips start counting in main loop
    for (int i = 0; i < MAX_RESCUES_PER_ROBOT; i++) {
        robot->survivors_saved[i] = -1;                                   //-1 means empty slot
    }
    
                                                                              //state machine init
    robot->state = ROBOT_STATE_IDLE;                                      //start idle at base
    robot->path_step = 0;                                                 //current step index in current_path
    robot->needs_replan = 1;                                              //force first planning call
    
                                                                              //sensor outputs
    robot->heat_detected = 0;                                             //no heat signal yet
    robot->co2_detected = 0;                                              //no co2 signal yet
    
                                                                              //statistics counters
    robot->cells_discovered = 0;                                          //how many cells this robot discovered
    robot->survivors_detected = 0;                                        //how many survivors this robot visually confirmed
    
    robot->total_time = 0;                                                //filled at end
    robot->start_time = 0;                                                 //filled when process starts
    robot->steps_taken = 0;                                               //movement/action steps
    
                                                                      //initialize local knowledge maps: everything unknown until the robot sees it
    for (int z = 0; z < MAX_Z; z++) {
        for (int y = 0; y < MAX_Y; y++) {
            for (int x = 0; x < MAX_X; x++) {
                robot->known_map[x][y][z] = CELL_UNKNOWN;                 //robot has no info yet
                robot->exploration_map[x][y][z] = 0;                      //can be used for exploration scoring
            }
        }
    }
    
                                                                          //robot knows the entry cell it starts on so it doesn't treat it as unknown
    robot->known_map[start_pos.x][start_pos.y][start_pos.z] = CELL_ENTRY; //mark start cell as entry
}

                                                                          //entry point for the robot process (each robot runs its own loop)
void robot_process_main(int robot_id, SharedData* shared, IPCResources* ipc,
                        const Config* config) {
    Robot* robot = &shared->robots[robot_id];                              //get robot struct in shared memory
    
                                                                              //seed randomness differently per robot so they don't behave identically
    utils_seed_random(config->random_seed + robot_id * 1000 + getpid());
    
    utils_log(LOG_INFO,
              "Robot %d process started (PID: %d) at base (%d,%d,%d)",
              robot_id, getpid(),
              robot->base_pos.x, robot->base_pos.y, robot->base_pos.z);   //print start info
    
                                                                              //store runtime fields now that process exists
    robot->pid = getpid();                                                 //save process id
    robot->start_time = get_current_time();                               //start time for elapsed computations
    robot->is_active = 1;                                                  //robot is officially active
    
                                                                            //signal readiness using semaphore to avoid race condition on shared->ready_flags
    sem_wait_op(ipc->sem_id, SEM_MUTEX);                                  //lock shared flags
    shared->ready_flags[robot_id] = 1;                                    //mark this robot ready
    sem_signal_op(ipc->sem_id, SEM_MUTEX);                                //unlock
    
                                                                        //wait until all robots have signaled ready (or terminate / timeout)
    int all_ready = 0;                                                    //flag for barrier
    int wait_count = 0;                                                   //limit waiting
    while (!all_ready && !shared->terminate_flag && wait_count < 100) {
        usleep(10000);                                                    //small sleep to avoid busy spinning
        wait_count++;
        all_ready = 1;                                                    //assume ready until proven otherwise
        for (int i = 0; i < shared->num_robots; i++) {
            if (!shared->ready_flags[i]) {                                 //if any robot not ready
                all_ready = 0;
                break;
            }
        }
    }
    
    utils_log(LOG_INFO,
              "Robot %d starting missions (will make multiple trips)",
              robot_id);                                                  //robot begins missions
    
                                                                        //main behavior loop: explore -> approach -> rescue -> return -> refill -> repeat
    robot_main_loop(robot, shared, ipc, config);
    
                                                                         //mark robot as finished (protected by mutex because shared counters are updated)
    sem_wait_op(ipc->sem_id, SEM_MUTEX);
    robot->is_finished = 1;                                               //finished flag
    robot->is_active = 0;                                                 //no longer active
    shared->robots_finished++;                                            //global finished count
    robot->total_time = get_elapsed_time(robot->start_time);              //store final time
    sem_signal_op(ipc->sem_id, SEM_MUTEX);
    
    utils_log(LOG_INFO,
              "Robot %d finished. Time: %.2f sec, Trips: %d, Survivors saved: %d, Steps: %d, Cells discovered: %d",
              robot_id,
              robot->total_time,
              robot->total_trips,
              robot->num_survivors_saved,
              robot->steps_taken,
              robot->cells_discovered);                                   //final summary per robot
}

                                                                          //robot state machine main loop: keeps running until time ends or mission ends
void robot_main_loop(Robot* robot, SharedData* shared, IPCResources* ipc,
                     const Config* config) {
    double max_time = config->max_simulation_time;                         //stop condition (global time limit)
    int stuck_counter = 0;                                                 //counts consecutive loops with no progress
    int max_stuck = 30;                                                    //threshold to trigger random escape move
    
                                                                              //start first trip immediately with exploration behavior
    robot->state = ROBOT_STATE_EXPLORING;                                  //first mission: explore
    robot->total_trips = 1;                                                //count trip #1
    
                                                                              //main execution loop: each iteration runs one "tick" of behavior
    while (!shared->terminate_flag &&
           robot->state != ROBOT_STATE_FINISHED &&
           get_elapsed_time(robot->start_time) < max_time) {
        
                                                                                  //mission-level termination: if everyone is rescued, stop
        if (shared->survivors_rescued >= shared->num_survivors) {
            utils_log(LOG_INFO,
                      "Robot %d: All survivors rescued! Mission complete.",
                      robot->id);
            robot->state = ROBOT_STATE_FINISHED;
            break;
        }
        
                                                                                    //state machine switch: run one handler per tick
        switch (robot->state) {
            case ROBOT_STATE_IDLE:
                                                                                  //robot is at base doing nothing: decide whether to explore or refill
                robot_handle_idle(robot, shared, config);
                break;
                
            case ROBOT_STATE_EXPLORING:
                                                                                  //robot is exploring unknown space: scan + detect + move toward exploration target
                robot_handle_exploring(robot, shared, ipc, config);
                break;
                
            case ROBOT_STATE_APPROACHING:
                                                                                  //robot has a known survivor target and is moving toward it
                robot_handle_approaching(robot, shared, ipc, config);
                break;
                
            case ROBOT_STATE_RESCUING:
                                                                                  //robot is at survivor location and tries to perform the rescue action
                robot_handle_rescuing(robot, shared, ipc, config);
                break;
                
            case ROBOT_STATE_RETURNING:
                                                                            //robot is returning to base after rescuing or when supplies are empty
                robot_handle_returning(robot, shared, ipc, config);
                break;
                
            case ROBOT_STATE_REFILLING:
                                                                              //robot is at base refilling supplies and preparing next trip
                robot_handle_refilling(robot, shared, config);
                break;
                
            case ROBOT_STATE_FINISHED:
                                                                             //already finished, nothing to do
                break;
        }
        
                                                                            //handle incoming updates from other robots (rescues / discovered cells)
        robot_process_messages(robot, shared, ipc);
        
                                                                           //stuck detection: if steps_taken doesn't increase for long, force random move
        if (robot->state != ROBOT_STATE_IDLE &&
            robot->state != ROBOT_STATE_REFILLING &&
            robot->state != ROBOT_STATE_RESCUING) {
            
            static int last_steps[MAX_ROBOTS] = {0};                       //track last step per robot id
            if (robot->steps_taken == last_steps[robot->id]) {             //no progress this tick
                stuck_counter++;
                if (stuck_counter > max_stuck) {
                    utils_log(LOG_WARNING,
                              "Robot %d stuck in state %s, forcing random move",
                              robot->id,
                              robot_state_name(robot->state));             //warn + show state
                    
                    robot_random_move(robot, shared);                       //attempt to escape obstacle
                    robot->needs_replan = 1;                                //force path recompute
                    stuck_counter = 0;                                      //reset stuck counter
                }
            } else {
                stuck_counter = 0;                                          //progress happened, reset
                last_steps[robot->id] = robot->steps_taken;                  //update last steps snapshot
            }
        }
        
                                                                            //global pacing: slower speed => longer sleep, faster speed => shorter sleep
        usleep((int)(30000 / config->robot_speed));                          //simulate time per action tick
    }
}

                                                                          // state handlers

                                                                          //idle behavior: decide next state based on global completion and supplies
void robot_handle_idle(Robot* robot, SharedData* shared, const Config* config) {
    (void)config;                                                           //currently not used
    
                                                                              //if mission complete, stop robot
    if (shared->survivors_rescued >= shared->num_survivors) {
        robot->state = ROBOT_STATE_FINISHED;
        return;
    }
    
                                                                              //if robot has no supplies, it must go refill (or return to base first)
    if (!robot->has_supplies) {
        if (point3d_equals(robot->current_pos, robot->base_pos)) {          //already at base
            robot->state = ROBOT_STATE_REFILLING;
        } else {                                                            //not at base yet
            robot->state = ROBOT_STATE_RETURNING;                           //go back to base
            robot->needs_replan = 1;                                        //need path to base
        }
        return;
    }
    
                                                                              //if robot has supplies, start exploring again
    robot->state = ROBOT_STATE_EXPLORING;
    robot->needs_replan = 1;                                                //force planning for exploration
}

                                                                          //exploring behavior: sense environment, pick target, plan path, move step-by-step
void robot_handle_exploring(Robot* robot, SharedData* shared, IPCResources* ipc,
                            const Config* config) {
                                                                              //sense environment: update maps + detect survivors
    robot_sense(robot, shared, ipc, config);
    
                                                                              //try to pick the best detected survivor (closest + not assigned) as target
    int target_survivor = robot_find_target_survivor(robot, shared);
    
    if (target_survivor >= 0) {
        robot->current_survivor_id = target_survivor;                         //store chosen survivor id
        robot->state = ROBOT_STATE_APPROACHING;                             //switch to approach mode
        robot->needs_replan = 1;                                              //need a new plan to survivor
        
                                                                                  //mark survivor as assigned so other robots avoid duplicating work
        for (int d = 0; d < shared->num_detected; d++) {
            if (shared->detected_survivors[d].id == target_survivor) {
                shared->detected_survivors[d].assigned_robot = robot->id;   //assign to this robot
                break;
            }
        }
        
        utils_log(LOG_INFO,
                  "Robot %d targeting Survivor %d",
                  robot->id,
                  target_survivor);
        return;
    }
    
                                                                              //no survivor target found: continue exploration (ga decides path to frontier)
    robot_plan(robot, shared, config);
    
                                                                              //execute a single move step, if failed then force replanning next tick
    if (!robot_execute_step(robot, shared, ipc, config)) {
        robot->needs_replan = 1;
    }
}

                                                                          //approaching behavior: move toward a chosen survivor and re-check if target changes
void robot_handle_approaching(Robot* robot, SharedData* shared, IPCResources* ipc,
                              const Config* config) {
                                                                              //sense during movement: might learn new cells or detect new survivors
    robot_sense(robot, shared, ipc, config);
    
                                                                              //if the chosen survivor got rescued by another robot, abandon and go back exploring
    if (robot->current_survivor_id >= 0 &&
        shared->survivors[robot->current_survivor_id].is_rescued) {
        
        utils_log(LOG_INFO,
                  "Robot %d: Target survivor %d already rescued, finding new target",
                  robot->id,
                  robot->current_survivor_id);
        
        robot->current_survivor_id = -1;                                   //clear target
        robot->state = ROBOT_STATE_EXPLORING;                              //go explore again
        robot->needs_replan = 1;                                           //plan new exploration path
        return;
    }
    
                                                                              //if robot is exactly at survivor position, switch to rescuing state
    if (robot->current_survivor_id >= 0) {
        Point3D surv_pos = shared->survivors[robot->current_survivor_id].position;
        if (point3d_equals(robot->current_pos, surv_pos)) {
            robot->state = ROBOT_STATE_RESCUING;
            return;
        }
    }
    
                                                                              //plan toward current target and move one step
    robot_plan(robot, shared, config);
    
    if (!robot_execute_step(robot, shared, ipc, config)) {
        robot->needs_replan = 1;                                           //path blocked or invalid
    }
    
                                                                              //after moving, check if we ended up on survivor cell (extra safety)
    int survivor_id = robot_check_survivor(robot, shared);
    if (survivor_id >= 0 && robot->has_supplies) {
        robot->current_survivor_id = survivor_id;                           //lock onto this survivor
        robot->state = ROBOT_STATE_RESCUING;                                //start rescue
    }
}

                                                                          //rescuing behavior: perform rescue action then return to base for supplies
void robot_handle_rescuing(Robot* robot, SharedData* shared, IPCResources* ipc,
                           const Config* config) {
                                                                              //attempt rescue with semaphore protection (shared survivor state)
    if (robot_rescue_survivor(robot, shared, ipc)) {
        utils_log(LOG_INFO,
                  "Robot %d rescued Survivor %d! (Total: %d) Returning to base...",
                  robot->id,
                  robot->current_survivor_id,
                  robot->num_survivors_saved);
        
        robot->state = ROBOT_STATE_RETURNING;                              //go back after rescue
        robot->needs_replan = 1;                                           //need path to base
    } else {
                                                                             //rescue might fail if another robot got it first or state changed
        utils_log(LOG_WARNING,
                  "Robot %d: Rescue failed for Survivor %d",
                  robot->id,
                  robot->current_survivor_id);
        
        robot->current_survivor_id = -1;                                   //drop target
        robot->state = ROBOT_STATE_EXPLORING;                              //resume exploration
        robot->needs_replan = 1;                                           //plan again
    }
    
                                                                              //simulate time spent rescuing (config is in seconds-like scale)
    usleep((int)(config->rescue_time * 100000));
}

                                                                          //returning behavior: plan path back to base and keep sensing while walking back
void robot_handle_returning(Robot* robot, SharedData* shared, IPCResources* ipc,
                            const Config* config) {
                                                                              //sense while returning so we still contribute discovery
    robot_sense(robot, shared, ipc, config);
    
                                                                              //if already at base, go to refill state
    if (point3d_equals(robot->current_pos, robot->base_pos)) {
        utils_log(LOG_INFO,
                  "Robot %d arrived at base. Starting refill...",
                  robot->id);
        robot->state = ROBOT_STATE_REFILLING;
        return;
    }
    
                                                                              //plan a dedicated path to base
    robot_plan_return_to_base(robot, shared, config);
    
                                                                              //execute movement step toward base
    if (!robot_execute_step(robot, shared, ipc, config)) {
        robot->needs_replan = 1;                                           //recompute if blocked
    }
}

                                                                          //refilling behavior wait for refill time, then start next trip or finish mission
void robot_handle_refilling(Robot* robot, SharedData* shared, const Config* config) {
                                                                              //simulate refill action time (converts to microseconds)
    usleep((int)(config->refill_time * 100000));
    
    robot->has_supplies = 1;                                               //supplies restored
    robot->current_survivor_id = -1;                                       //clear target
    robot->total_trips++;                                                  //new trip begins after refill
    
    shared->stats.total_trips++;                                           //update global stats
    
    utils_log(LOG_INFO,
              "Robot %d refilled supplies. Starting trip #%d",
              robot->id,
              robot->total_trips);
    
                                                                              //if everything is already rescued, stop, otherwise explore again
    if (shared->survivors_rescued >= shared->num_survivors) {
        robot->state = ROBOT_STATE_FINISHED;
    } else {
        robot->state = ROBOT_STATE_EXPLORING;
        robot->needs_replan = 1;
    }
}

                                                                            //helper functions

                                                                          //choose a detected survivor to target: closest unrescued and not assigned to another robot
int robot_find_target_survivor(Robot* robot, SharedData* shared) {
    int best_id = -1;                                                       //best survivor id
    double best_dist = DBL_MAX;                                           //best distance so far
    
    for (int d = 0; d < shared->num_detected; d++) {
        DetectedSurvivor* ds = &shared->detected_survivors[d];              //detected survivor record
        int sid = ds->id;                                                 //actual survivor id
        
        if (shared->survivors[sid].is_rescued) continue;                    //skip if already rescued
        if (ds->assigned_robot != -1 && ds->assigned_robot != robot->id) continue; //skip if taken
        
                                                                               //distance from robot to estimated survivor position
        double dist = map_euclidean_distance(robot->current_pos, ds->estimated_pos);
        if (dist < best_dist) {                                           //choose closest
            best_dist = dist;
            best_id = sid;
        }
    }
    
    return best_id;                                                          //-1 means no suitable target
}

                                                                          //sense sync global knowledge, scan around robot (with map lock), then detect survivors
void robot_sense(Robot* robot, SharedData* shared, IPCResources* ipc, const Config* config) {
    discovery_sync_knowledge(robot, shared);                               //pull global discovered cells into robot map
    
    sem_wait_op(ipc->sem_id, SEM_MAP_ACCESS);                               //lock shared map for safe scanning updates
    discovery_scan(robot, shared, config);                                 //update discovered cells
    sem_signal_op(ipc->sem_id, SEM_MAP_ACCESS);                                //unlock map
    
    discovery_detect_survivors(robot, shared, config);                     //update sensor readings and detections
}

                                                                             //plan using online ga only when required (saves time)
void robot_plan(Robot* robot, SharedData* shared, const Config* config) {
    if (!online_ga_needs_replan(robot, shared, config)) {                  //skip if current plan is still good
        return;
    }
    
    online_ga_plan(robot, shared, config);                                 //compute/update robot->current_path
    shared->stats.total_replans++;                                           //stats tracking for analysis
}

                                                                          //plan a return path specifically to base position
void robot_plan_return_to_base(Robot* robot, SharedData* shared, const Config* config) {
    robot->current_path.target_type = 2;                                   //2 means return-to-base target
    robot->current_path.target_pos = robot->base_pos;                      //store base coordinates
    
                                                                              //if path is outdated or finished, generate a fresh one
    if (robot->needs_replan || robot->path_step >= robot->current_path.length) {
        Chromosome path;                                                   //temporary generated path
        if (online_ga_generate_rescue_path(&path, robot, robot->base_pos, shared, config)) {
            memcpy(&robot->current_path, &path, sizeof(Chromosome));        //copy new path to robot
            robot->path_step = 0;                                          //start from first gene
            robot->needs_replan = 0;                                        //planning done
        }
    }
}

                                                                            //execute one movement step along current_path, with safety checks for debris/invalid path
int robot_execute_step(Robot* robot, SharedData* shared, IPCResources* ipc,
                       const Config* config) {
    (void)ipc;                                                             //ipc not used inside this function
    (void)config;                                                           //config not used inside this function
    
    if (!robot->current_path.is_valid || robot->current_path.length == 0) { //no valid plan
        return 0;
    }
    
    if (robot->path_step >= robot->current_path.length) {                  //already at end of path
        robot->needs_replan = 1;
        return 0;
    }
    
    Point3D next_pos = robot->current_path.genes[robot->path_step].position; //read next step position
    
                                                                              //use discovery aware cell readc an return unknown or debris
    int cell = discovery_get_cell(robot, shared, next_pos.x, next_pos.y, next_pos.z);
    if (cell == CELL_DEBRIS) {                                             //planned step is blocked
        robot->needs_replan = 1;
        return 0;
    }
    
                                                                              //try to actually move (checks real grid and updates knowledge)
    if (!robot_move(robot, shared, next_pos)) {
        robot->needs_replan = 1;
        return 0;
    }
    
    robot->path_step++;                                                    //advance in path
    robot->steps_taken++;                                                   //count progress
    
    return 1;                                                              //success
}

                                                                           //random movement used as emergency escape if robot becomes stuck
void robot_random_move(Robot* robot, SharedData* shared) {
    Point3D pos = robot->current_pos;                                      //current position snapshot
    
    int directions[NUM_DIRECTIONS];                                        //direction indices list
    for (int i = 0; i < NUM_DIRECTIONS; i++) {
        directions[i] = i;                                                 //fill 0 to NUM_DIRECTIONS-1
    }
    
                                                                              //shuffle directions so the robot tries moves in random order
    for (int i = NUM_DIRECTIONS - 1; i > 0; i--) {
        int j = utils_random_int(0, i);
        int tmp = directions[i];
        directions[i] = directions[j];
        directions[j] = tmp;
    }
    
                                                                            //try each shuffled direction until a valid non-debris cell is found
    for (int i = 0; i < NUM_DIRECTIONS; i++) {
        Point3D next = utils_apply_direction(pos, directions[i]);          //apply direction delta
        
                                                                           //bounds check so we never move outside the map
        if (next.x < 0 || next.x >= shared->dim_x ||
            next.y < 0 || next.y >= shared->dim_y ||
            next.z < 0 || next.z >= shared->dim_z) {
            continue;
        }
        
                                                                                  //skip debris cells 
        if (shared->grid[next.x][next.y][next.z] != CELL_DEBRIS) {
            robot->current_pos = next;                                     //commit move
            robot->steps_taken++;                                            //count step
            
                                                                           //update robot knowledge and shared discovered map to reflect this cell
            robot->known_map[next.x][next.y][next.z] =
                shared->grid[next.x][next.y][next.z];
            
            if (shared->discovered[next.x][next.y][next.z] == CELL_UNKNOWN) {
                shared->discovered[next.x][next.y][next.z] =
                    shared->grid[next.x][next.y][next.z];
                shared->total_discovered++;
            }
            return;                                                        //stop after first valid move
        }
    }
}

                                                                          //move robot to a new position, update knowledge maps, and reject illegal/debris moves
int robot_move(Robot* robot, SharedData* shared, Point3D new_pos) {
    //bounds check for safety
    if (new_pos.x < 0 || new_pos.x >= shared->dim_x ||
        new_pos.y < 0 || new_pos.y >= shared->dim_y ||
        new_pos.z < 0 || new_pos.z >= shared->dim_z) {
        return 0;
    }
    
    int actual_cell = shared->grid[new_pos.x][new_pos.y][new_pos.z];       //true cell value from global grid
    
                                                                              //if debris, block movement and also update knowledge so robot doesn't keep trying it
    if (actual_cell == CELL_DEBRIS) {
        robot->known_map[new_pos.x][new_pos.y][new_pos.z] = CELL_DEBRIS;   //robot learns it's blocked
        shared->discovered[new_pos.x][new_pos.y][new_pos.z] = CELL_DEBRIS; //global discovery also updated
        robot->needs_replan = 1;                                           //path is invalid now
        return 0;
    }
    
    robot->current_pos = new_pos;                                          //commit move
    
                                                                              //update robot local discovery stats when entering a new unknown cell
    if (robot->known_map[new_pos.x][new_pos.y][new_pos.z] == CELL_UNKNOWN) {
        robot->known_map[new_pos.x][new_pos.y][new_pos.z] = actual_cell;   //store cell type
        robot->cells_discovered++;                                         //count as discovered by this robot
    }
    
                                                                              //update global discovery map if this cell was unknown globally
    if (shared->discovered[new_pos.x][new_pos.y][new_pos.z] == CELL_UNKNOWN) {
        shared->discovered[new_pos.x][new_pos.y][new_pos.z] = actual_cell;
        shared->total_discovered++;
    }
    
    return 1;                                                              //movement succeeded
}

                                                                             //check if robot is currently standing on a survivor location
int robot_check_survivor(Robot* robot, SharedData* shared) {
    for (int s = 0; s < shared->num_survivors; s++) {                      //scan all survivors
        if (!shared->survivors[s].is_rescued &&
            point3d_equals(robot->current_pos, shared->survivors[s].position)) {
            return s;                                                      //found survivor id
        }
    }
    return -1;                                                             //no survivor at current cell
}

                                                                             //rescue survivor with semaphore protection and broadcast the event to other robots
int robot_rescue_survivor(Robot* robot, SharedData* shared, IPCResources* ipc) {
    if (!robot->has_supplies) {                                            //cannot rescue without supplies
        return 0;
    }
    
    int survivor_id = robot->current_survivor_id;                            //preferred target id
    if (survivor_id < 0) {
        survivor_id = robot_check_survivor(robot, shared);                  //fallback: check current cell
    }
    
    if (survivor_id < 0) {                                                 //still nothing to rescue
        return 0;
    }
    
                                                                              //lock survivor data: multiple robots may try rescuing same survivor at same time
    sem_wait_op(ipc->sem_id, SEM_SURVIVOR);
    
    if (shared->survivors[survivor_id].is_rescued) {                        //someone already rescued it
        sem_signal_op(ipc->sem_id, SEM_SURVIVOR);
        return 0;
    }
    
                                                                              //perform rescue: update survivor record
    shared->survivors[survivor_id].is_rescued = 1;                          //mark rescued
    shared->survivors[survivor_id].rescued_by_robot = robot->id;            //store rescuer id
    shared->survivors[survivor_id].rescue_time =
        get_elapsed_time(robot->start_time);                                //time since robot started
    
                                                                               //update robot state and bookkeeping
    robot->has_supplies = 0;                                                //supplies used up by rescue
    robot->survivors_saved[robot->num_survivors_saved] = survivor_id;       //record survivor
    robot->num_survivors_saved++;                                           //increment saved count
    
                                                                              //update map cell visually/statefully
    Point3D surv_pos = shared->survivors[survivor_id].position;
    shared->grid[surv_pos.x][surv_pos.y][surv_pos.z] = CELL_RESCUED;          //change cell to rescued marker
    shared->survivors_rescued++;                                             //global rescued counter
    
    sem_signal_op(ipc->sem_id, SEM_SURVIVOR);                               //unlock survivor data
    
                                                                               //broadcast so other robots can replan and avoid targeting same survivor
    robot_broadcast_rescue(robot, survivor_id, ipc);
    
    utils_log(LOG_INFO,
              "Robot %d rescued survivor %d at (%d,%d,%d)! [%d/%d total]",
              robot->id, survivor_id,
              surv_pos.x, surv_pos.y, surv_pos.z,
              shared->survivors_rescued, shared->num_survivors);
    
    return 1;
}

                                                                            //send rescue event message to all other robots using message queue
void robot_broadcast_rescue(Robot* robot, int survivor_id, IPCResources* ipc) {
    IPCMessage msg;
    msg.mtype = MSG_SURVIVOR_RESCUED;                                      //message type for kernel routing
    msg.sender_id = robot->id;                                             //who sent it
    msg.receiver_id = -1;                                                  //broadcast
    msg.message_type = MSG_SURVIVOR_RESCUED;                                //payload type
    msg.data_int = survivor_id;                                             //survivor id as data
    msg.location = robot->current_pos;                                      //where rescue happened
    msg.timestamp = get_current_time();                                    //timestamp
    
    SharedData* shared = ipc->shared_data;                                 //shared pointer from ipc container
    for (int i = 0; i < shared->num_robots; i++) {                         //send to each robot
        if (i != robot->id) {                                              //skip self
            msg.mtype = i + 100;                                           //robot-specific mailbox
            msgsnd(ipc->msg_id, &msg, sizeof(IPCMessage) - sizeof(long), IPC_NOWAIT);
        }
    }
}

                                                                           //process messages for this robot (currently delegates to discovery message processing)
void robot_process_messages(Robot* robot, SharedData* shared, IPCResources* ipc) {
    discovery_process_messages(robot, shared, ipc->msg_id);                //handle discovered cells + rescue notifications
}

                                                                           //convert robot state enum value to readable string for logs
const char* robot_state_name(int state) {
    switch (state) {
        case ROBOT_STATE_IDLE:        return "IDLE";
        case ROBOT_STATE_EXPLORING:   return "EXPLORING";
        case ROBOT_STATE_APPROACHING: return "APPROACHING";
        case ROBOT_STATE_RESCUING:    return "RESCUING";
        case ROBOT_STATE_RETURNING:   return "RETURNING";
        case ROBOT_STATE_REFILLING:   return "REFILLING";
        case ROBOT_STATE_FINISHED:    return "FINISHED";
        default: return "UNKNOWN";
    }
}
