//discovery module implementation of map exploration, survivor detection, and knowledge sharing

#include "discovery.h"   //discovery api and shared types
#include "utils.h"          //logging, random, timing helpers
#include "map.h"          //map utilities distance, boundaries, etc
#include "common.h"      //shared constants, cell types, directions
#include "online_ga.h"    //ga hooks 

                                                                         //initialize shared discovery buffers and counters
void discovery_init(SharedData* shared, const Config* config) {
    (void)config;                                                     //config currently unused here
    
    for (int z = 0; z < shared->dim_z; z++) {     
                                                                      //loop z layers
        for (int y = 0; y < shared->dim_y; y++) {     
                                                                     //loop rows
            for (int x = 0; x < shared->dim_x; x++) {     
                                                                     //loop columns
                shared->discovered[x][y][z] = CELL_UNKNOWN;  
                                                                     //mark cell as unknown globally
                shared->discovered_by[x][y][z] = -1;                   //no robot discovered it yet
            }
        }
    }
    
    shared->total_cells = shared->dim_x * shared->dim_y * shared->dim_z; //total number of cells
    shared->total_discovered = 0;  
                                                                        //global discovered count
    shared->num_detected = 0;                                            //detected survivor entries count
    shared->survivors_detected = 0;                                     //how many survivors have been confirmed
    
                                                                           //mark entry points as already known
    for (int i = 0; i < shared->num_entry_points; i++) { 
                                                                         //loop entry points
        Point3D p = shared->entry_points[i].position;    
                                                                       //entry position
        shared->discovered[p.x][p.y][p.z] = shared->grid[p.x][p.y][p.z];//copy real cell type into discovered map
        shared->total_discovered++;                                    //increment discovered cells
    }
    
    utils_log(LOG_INFO, "Discovery system initialized. %d cells to explore.",
              shared->total_cells);                                    //log initialization info
}

                                                                         //scan around the robot and mark visible cells as discovered
void discovery_scan(Robot* robot, SharedData* shared, const Config* config) {
    Point3D pos = robot->current_pos;   
                                                                       //robot current position
    int range = config->visual_range;                                  //visual scan radius
    
    for (int dz = -range; dz <= range; dz++) {   
                                                                       //scan cube in z
        for (int dy = -range; dy <= range; dy++) {   
                                                                       //scan cube in y
            for (int dx = -range; dx <= range; dx++) {  
                                                                        //scan cube in x
                int x = pos.x + dx;   
                                                                        //candidate x
                int y = pos.y + dy;   
                                                                       //candidate y
                int z = pos.z + dz;                                    //candidate z
                
                if (x < 0 || x >= shared->dim_x ||                     //x bounds check
                    y < 0 || y >= shared->dim_y ||                      //y bounds check
                    z < 0 || z >= shared->dim_z) {                     //z bounds check
                    continue;                                           //skip out of bounds
                }
                
                double dist = sqrt(dx*dx + dy*dy + dz*dz);               //euclidean distance
                if (dist > range) continue;                            //keep spherical range
                
                                                                         //discover cell for this robot
                if (robot->known_map[x][y][z] == CELL_UNKNOWN) {   
                                                                         //only if robot doesn't know it yet
                    int cell_type = shared->grid[x][y][z];     
                                                                         //read real map value
                    robot->known_map[x][y][z] = cell_type; 
                                                                         //store in robot knowledge
                    robot->cells_discovered++;                           //robot local counter
                    
                                                                         //update global discovered map 
                    if (shared->discovered[x][y][z] == CELL_UNKNOWN) { //first global discovery
                        shared->discovered[x][y][z] = cell_type;   
                                                                              //store discovered type
                        shared->discovered_by[x][y][z] = robot->id;    
                                                                             //store discoverer robot id
                        shared->total_discovered++;                        //increment global discovered count
                    }
                }
            }
        }
    }
}

                                                                         //get a cell type using robot knowledge then global discovery otherwise unknown
int discovery_get_cell(Robot* robot, SharedData* shared, int x, int y, int z) {
    if (x < 0 || x >= shared->dim_x ||                                 //x bounds
        y < 0 || y >= shared->dim_y ||                                  //y bounds
        z < 0 || z >= shared->dim_z) {      
                                                                          //z bounds
        return CELL_DEBRIS;                                              //treat out of bounds as blocked
    }
    
    if (robot->known_map[x][y][z] != CELL_UNKNOWN) {    
                                                                           //robot already knows
        return robot->known_map[x][y][z];                               //return robot known type
    }
    
    if (shared->discovered[x][y][z] != CELL_UNKNOWN) { 
                                                                         //global map knows it
        robot->known_map[x][y][z] = shared->discovered[x][y][z]; 
                                                                       //sync into robot map
        return shared->discovered[x][y][z];                             //return known type
    }
    
    return CELL_UNKNOWN;                                               //still unknown
}

                                                                          //detect survivors using sensors and direct visual confirmation
int discovery_detect_survivors(Robot* robot, SharedData* shared, const Config* config) {
    Point3D pos = robot->current_pos;                                  //robot current position
    int detected = 0;                                                    //new detections count
    
    robot->heat_detected = 0; 
                                                                           //reset best heat signal
    robot->co2_detected = 0;                                             //reset best co2 signal
    
    for (int s = 0; s < shared->num_survivors; s++) {
                                                                           //scan all survivors
        if (shared->survivors[s].is_rescued) {                            //skip already rescued
            continue;
        }
        
        Point3D surv_pos = shared->survivors[s].position; 
                                                                         //survivor position
        double dist = map_euclidean_distance(pos, surv_pos);             //distance to survivor
        
                                                               //heat detection
        if (dist <= config->heat_sensor_range && dist > 0) {             //within heat range
            int signal = (int)(shared->survivors[s].heat_level *       //signal strength
                              (1.0 - dist / config->heat_sensor_range));
            if (signal > robot->heat_detected) {    
                                                                       //keep strongest signal
                robot->heat_detected = signal;    
                                                                        //store strength
                robot->heat_source = surv_pos;                          //store source location
            }
        }
        
                                                                             //co2 detection
        if (dist <= config->co2_sensor_range && dist > 0) {            //within co2 range
            int signal = (int)(shared->survivors[s].co2_level *         //signal strength
                              (1.0 - dist / config->co2_sensor_range));
            if (signal > robot->co2_detected) {                            //keep strongest signal
                robot->co2_detected = signal;    
                                                                          //store strength
                robot->co2_source = surv_pos;                           //store source location
            }
        }
        
                                                                               //visual confirmation
        if (dist <= config->visual_range) {                            //within vision range
            if (!shared->survivors[s].is_detected) {                   //not confirmed before
                shared->survivors[s].is_detected = 1;                  //mark as detected
                
                                                                         //add to detected survivors list
                if (shared->num_detected < MAX_SURVIVORS) {            //capacity check
                    DetectedSurvivor* ds =
                        &shared->detected_survivors[shared->num_detected]; //next slot
                    
                    ds->id = s;                                        //survivor id
                    ds->estimated_pos = surv_pos; 
                                                                  //estimated/confirmed position
                    ds->heat_signal = shared->survivors[s].heat_level; //store heat reading
                    ds->co2_signal = shared->survivors[s].co2_level;   //store co2 reading
                    ds->confirmed = 1;          
                                                                    //visual confirmation flag
                    ds->assigned_robot = -1;                           //no assignment yet
                    ds->detection_time = get_current_time();             //timestamp detection
                    
                    shared->num_detected++;                            //increment detected list count
                    shared->survivors_detected++;                        //increment confirmed survivors
                    
                    detected++;                                        //count this detection
                    robot->survivors_detected++;                         //robot counter
                    robot->needs_replan = 1;                           //force replanning
                    
                    utils_log(LOG_INFO,
                              "Robot %d DETECTED Survivor %d at (%d,%d,%d)!\n",
                              robot->id, s, surv_pos.x, surv_pos.y, surv_pos.z); //log detection
                }
            }
        }
    }
    
    return detected;                                                   //return number of new detections
}

                                                                         //receive ipc messages relevant to discovery and update robot/shared state
void discovery_process_messages(Robot* robot, SharedData* shared, int msg_id) {
    IPCMessage msg;                                                    //message buffer
    
    while (msgrcv(msg_id, &msg, sizeof(IPCMessage) - sizeof(long),
                  robot->id + 100, IPC_NOWAIT) > 0) {                   //non blocking receive
        
        if (msg.message_type == MSG_CELL_DISCOVERED) {                   //cell discovery message
            int x = msg.location.x;                                    //msg x
            int y = msg.location.y;                                      //msg y
            int z = msg.location.z;                                    //msg z
            
            if (x >= 0 && x < shared->dim_x &&                         //bounds check
                y >= 0 && y < shared->dim_y &&
                z >= 0 && z < shared->dim_z) {
                if (robot->known_map[x][y][z] == CELL_UNKNOWN) {        //only if unknown
                    robot->known_map[x][y][z] = msg.cell_type;           //store learned cell type
                }
            }
        }
        else if (msg.message_type == MSG_SURVIVOR_RESCUED) {             //rescued event
            robot->needs_replan = 1;                                    //paths or targets changed
        }
        else if (msg.message_type == MSG_TERMINATE) {                  //terminate signal
            shared->terminate_flag = 1;                                  //stop simulation
        }
    }
}

                                                                          //pick a good next exploration target for a robot
Point3D discovery_find_exploration_target(Robot* robot, SharedData* shared,
                                          const Config* config) {
    (void)config;                                                      //not used currently
    
    Point3D best_target = robot->current_pos;                          //default target = stay
    double best_value = -1;                                            //best score so far
    
                                                                           //if we have sensor readings, move towards them
    if (robot->heat_detected > 30) {                                   //heat strong enough
        return robot->heat_source;                                       //go to heat source
    }
    if (robot->co2_detected > 30) {                                      //co2 strong enough
        return robot->co2_source;                                      //go to co2 source
    }
    
                                                                           //search for frontier cells known cells adjacent to unknown
    int search_range = 15;                                             //local search window size
    
    for (int dz = -search_range; dz <= search_range; dz++) {   
                                                                          //search cube z
        for (int dy = -search_range; dy <= search_range; dy++) {        //search cube y
            for (int dx = -search_range; dx <= search_range; dx++) { 
                                                                        //search cube x
                int x = robot->current_pos.x + dx;   
                                                                        //candidate x
                int y = robot->current_pos.y + dy;      
                                                                       //candidate y
                int z = robot->current_pos.z + dz;                        //candidate z
                
                if (x < 0 || x >= shared->dim_x ||                     //bounds
                    y < 0 || y >= shared->dim_y ||
                    z < 0 || z >= shared->dim_z) {
                    continue;
                }
                
                int cell = robot->known_map[x][y][z];                   //robot known cell
                
                                                                          //must be known and not debris
                if (cell == CELL_UNKNOWN || cell == CELL_DEBRIS) {    
                                                                       //skip unknown/blocked
                    continue;
                }
                
                                                                         //check if adjacent to unknown frontier
                int has_unknown = 0;                                     //flag for frontier
                for (int d = 0; d < 8 && !has_unknown; d++) {   
                                                                         //use 8 neighborhood in x and y
                    int nx = x + DIR_X[d];                                //neighbor x
                    int ny = y + DIR_Y[d];                               //neighbor y
                    int nz = z;                                          //same layer
                    
                    if (nx >= 0 && nx < shared->dim_x &&               //bounds
                        ny >= 0 && ny < shared->dim_y &&
                        robot->known_map[nx][ny][nz] == CELL_UNKNOWN) { //neighbor unknown
                        has_unknown = 1;                               //frontier found
                    }
                }
                
                if (!has_unknown) continue;                            //skip non frontier
                
                double dist = map_euclidean_distance(robot->current_pos,
                                                     point3d_create(x, y, z)); //distance to candidate
                
                                                                             //scoring: prefer closer cells + small randomness
                double value = 100.0 - dist;                           //closer then higher score
                value += utils_random_double(-5, 5);                   //add exploration noise
                
                if (value > best_value) {                                 //keep best candidate
                    best_value = value;                                 //update best score
                    best_target = point3d_create(x, y, z);               //store best target
                }
            }
        }
    }
    
                                                                            //if no frontier found and pick a random known accessible cell
    if (best_value < 0) {                                              //no frontier
        for (int attempt = 0; attempt < 100; attempt++) {              //limit attempts
            int x = utils_random_int(0, shared->dim_x - 1);              //random x
            int y = utils_random_int(0, shared->dim_y - 1);             //random y
            int z = utils_random_int(0, shared->dim_z - 1);             //random z
            
            int cell = robot->known_map[x][y][z];                      //robot known cell
            if (cell != CELL_UNKNOWN && cell != CELL_DEBRIS) {          //must be traversable
                return point3d_create(x, y, z);                         //return random reachable
            }
        }
    }
    
    return best_target;                                                //return chosen target
}

                                                                         //return discovery completion percentage
double discovery_get_progress(SharedData* shared) {
    if (shared->total_cells == 0) return 100.0;                          //avoid divide by zero
    return 100.0 * shared->total_discovered / shared->total_cells;         ///percentage explored
}

                                                                        //sync robot local known map with the global discovered map
void discovery_sync_knowledge(Robot* robot, SharedData* shared) {
    for (int z = 0; z < shared->dim_z; z++) {                           //loop layers
        for (int y = 0; y < shared->dim_y; y++) {                      //loop rows
            for (int x = 0; x < shared->dim_x; x++) {                   //loop cols
                if (robot->known_map[x][y][z] == CELL_UNKNOWN &&        //robot doesn't know
                    shared->discovered[x][y][z] != CELL_UNKNOWN) {     //global knows
                    robot->known_map[x][y][z] = shared->discovered[x][y][z]; //copy knowledge
                }
            }
        }
    }
}
 