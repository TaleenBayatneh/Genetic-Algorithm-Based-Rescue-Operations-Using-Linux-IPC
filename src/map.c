#include "map.h"
#include "utils.h"

//////////////////////////////////////////////////////////
//initialize all map related data structures and simulation state.
//this function prepares the shared memory before map generation.
void map_init(SharedData* shared, const Config* config) {
    //to make sure that the shared memory and config are valid
    if (!shared || !config) return;
    
    //copy map dimensions from config to the shared memory
    shared->dim_x = config->map_width;
    shared->dim_y = config->map_height;
    shared->dim_z = config->map_depth;
    
    //survivor related counters
    shared->num_survivors = 0;//total number of survivors placed 
    shared->survivors_rescued = 0;//number of rescued survivors
    shared->survivors_detected = 0;//number of detected survivors
    shared->num_detected = 0;
    
    //robot related counters
    shared->num_robots = config->num_robots;//total robots in the simulation
    shared->robots_finished = 0;//robots that completed tasks
    shared->num_entry_points = 0;//entry points found on map
    
    //simulation control flags
    shared->simulation_running = 0;//simulation not started yet
    shared->simulation_paused = 0;//simulation not paused
    shared->terminate_flag = 0;//no termination requested
    
    //calculate total number of cells in the map
    shared->total_cells = shared->dim_x * shared->dim_y * shared->dim_z;
    shared->total_discovered = 0;//total cells discovered
    

    //Initialize all shared arrays
    memset(shared->grid, 0, sizeof(shared->grid)); //ground truth map which is the actual environment
    memset(shared->risk_map, 0, sizeof(shared->risk_map));//risk values per cell
    memset(shared->discovered, CELL_UNKNOWN, sizeof(shared->discovered));//shared discovery map
    memset(shared->discovered_by, -1, sizeof(shared->discovered_by));//track which robot discovered each cell (-1 = none)
    memset(shared->survivors, 0, sizeof(shared->survivors));//survivors array
    memset(shared->detected_survivors, 0, sizeof(shared->detected_survivors));//detected survivors list
    memset(shared->robots, 0, sizeof(shared->robots));//robot data array
    memset(shared->entry_points, 0, sizeof(shared->entry_points));//entry points list
    memset(shared->ready_flags, 0, sizeof(shared->ready_flags));//robot readiness flags
    memset(&shared->stats, 0, sizeof(shared->stats));//statistics structure
    
    //explicitly mark all discoverd cells as unknown 
    for (int z = 0; z < MAX_Z; z++) {
        for (int y = 0; y < MAX_Y; y++) {
            for (int x = 0; x < MAX_X; x++) {
                shared->discovered[x][y][z] = CELL_UNKNOWN;
            }
        }
    }
    
    //log successful map initialization
    utils_log(LOG_INFO, "Map initialized: %dx%dx%d", shared->dim_x, shared->dim_y, shared->dim_z);
}


//////////////////////////////////////////////////////////
//generate the map content:
//place debris
//identify entry points
//place survivors
//calculate risk levels
void map_generate(SharedData* shared, const Config* config) {
    //to make sure that the shared memory and config are valid
    if (!shared || !config) return;
    
    // log the start of map generation with debris percentage
    utils_log(LOG_INFO, "Generating map with %.0f%% debris density", config->debris_density * 100);
    
    //Generate debris
    for (int z = 0; z < shared->dim_z; z++) {
        for (int y = 0; y < shared->dim_y; y++) {
            for (int x = 0; x < shared->dim_x; x++) {
                //check if the current cell is on the boundary of the map
                if (map_is_boundary(shared, x, y, z)) {
                    //reduce debris density on boundary cells
                    if (utils_random_double(0, 1) > config->debris_density * 0.5) {
                        //boundary cell is empty
                        shared->grid[x][y][z] = CELL_EMPTY;
                    } else {
                        //boundary cell contains debris
                        shared->grid[x][y][z] = CELL_DEBRIS;
                    }
                } else {
                    //interior cells use full debris density
                    if (utils_random_double(0, 1) < config->debris_density) {
                        //interior cell contains debris
                        shared->grid[x][y][z] = CELL_DEBRIS;
                    } else {
                        //interior cell is empty
                        shared->grid[x][y][z] = CELL_EMPTY;
                    }
                }
            }
        }
    }
    
    map_find_entry_points(shared);//identify valid entry points on the map boundaries
    map_place_survivors(shared, config);//place survivors in accessible empty locations
    map_calculate_risks(shared, config);//compute risk values for all cells
    
    //log summary of generated map
    utils_log(LOG_INFO, "Map generated with %d survivors and %d entry points",
              shared->num_survivors, shared->num_entry_points);
}


//////////////////////////////////////////////////////////
//randomly place survivors in accessible empty cells
//survivors must have at least one free neighboring cell
void map_place_survivors(SharedData* shared, const Config* config) {
    int placed = 0;//number of successfully placed survivors
    int attempts = 0;//number of placement attempts
    int max_attempts = config->num_survivors * 100; //maximum allowed attempts to place all survivors
    
    //try to place survivors until all are placed or attempts run out
    while (placed < config->num_survivors && attempts < max_attempts) {
        //generate a random position inside the map (not on boundaries)
        int x = utils_random_int(1, shared->dim_x - 2);
        int y = utils_random_int(1, shared->dim_y - 2);
        int z = utils_random_int(0, shared->dim_z - 1);
        
        //check if the selected cell is empty
        if (shared->grid[x][y][z] == CELL_EMPTY) {
            int accessible = 0;
            //check all neighboring cells for accessibility
            for (int d = 0; d < NUM_DIRECTIONS && !accessible; d++) {
                int nx = x + DIR_X[d];
                int ny = y + DIR_Y[d];
                int nz = z + DIR_Z[d];

                //neighbor must be inside the map and empty
                //at least one neighbor should be empty to make accessible
                if (map_is_valid_cell(shared, nx, ny, nz) && 
                    shared->grid[nx][ny][nz] == CELL_EMPTY) {
                    accessible = 1;
                }
            }
            
            //place survivor only if the location is accessible
            if (accessible) {
                //mark the cell as containing a survivor
                shared->grid[x][y][z] = CELL_SURVIVOR;
                // Initialize survivor data structure
                Survivor* s = &shared->survivors[placed];
                s->id = placed;
                s->position = point3d_create(x, y, z);
                s->is_alive = 1;
                s->is_rescued = 0;
                s->is_detected = 0;
                s->rescued_by_robot = -1;
                s->rescue_time = 0;
                s->heat_level = utils_random_int(50, 100);
                s->co2_level = utils_random_int(30, 90);
                s->priority = (s->heat_level + s->co2_level) / 2;
                
                placed++;
            }
        }
        attempts++;
    }
    
    //update global survivor count
    shared->num_survivors = placed;
    shared->stats.total_survivors = placed;
    
    //warn if not all survivors were placed
    if (placed < config->num_survivors) {
        utils_log(LOG_WARNING, "Only placed %d/%d survivors", placed, config->num_survivors);
    }
}


//////////////////////////////////////////////////////////
//compute a risk value for each cell based on nearby debris and floor level (higher floors are riskier)
void map_calculate_risks(SharedData* shared, const Config* config) {
    for (int z = 0; z < shared->dim_z; z++) {
        for (int y = 0; y < shared->dim_y; y++) {
            for (int x = 0; x < shared->dim_x; x++) {
                //initialize risk value for the current cell
                int risk = 0;
                //if the cell itself contains debris so it is extremely dangerous
                if (shared->grid[x][y][z] == CELL_DEBRIS) {
                    risk = 100;
                } else {
                    int debris_count = 0;
                    //count neighboring debris cells
                    for (int d = 0; d < NUM_DIRECTIONS; d++) {
                        int nx = x + DIR_X[d];
                        int ny = y + DIR_Y[d];
                        int nz = z + DIR_Z[d];
                        //check if neighbor is inside map and is debris
                        if (map_is_valid_cell(shared, nx, ny, nz) &&
                            shared->grid[nx][ny][nz] == CELL_DEBRIS) {
                            debris_count++;
                        }
                    }
                    //risk contribution from nearby debris
                    risk = (int)(debris_count * config->debris_risk * 10);
                    //add extra risk for deeper levels
                    risk += (shared->dim_z - z - 1) * 5;
                }
                
                //cap the risk value to a maximum of 100
                shared->risk_map[x][y][z] = risk < 100 ? risk : 100;
            }
        }
    }
}


//////////////////////////////////////////////////////////
//
void map_find_entry_points(SharedData* shared) {
    shared->num_entry_points = 0;
    
    for (int z = 0; z < shared->dim_z; z++) {
        for (int y = 0; y < shared->dim_y; y++) {
            for (int x = 0; x < shared->dim_x; x++) {
                //check if the cell is on the boundary and is empty
                if (map_is_boundary(shared, x, y, z) && 
                    shared->grid[x][y][z] == CELL_EMPTY) {
                    //make sure we do not exceed the maximum allowed entry points
                    if (shared->num_entry_points < MAX_ENTRY_POINTS) {
                        //store entry point position
                        shared->entry_points[shared->num_entry_points].position = 
                            point3d_create(x, y, z);
                        shared->entry_points[shared->num_entry_points].is_available = 1;//mark entry point as available
                        shared->grid[x][y][z] = CELL_ENTRY;//mark the map cell as an entry cell
                        shared->num_entry_points++;//increment entry point count
                    }
                }
            }
        }
    }
}


//////////////////////////////////////////////////////////
//check whether the given coordinates are inside the map boundaries
int map_is_valid_cell(const SharedData* shared, int x, int y, int z) {
    return x >= 0 && x < shared->dim_x &&
           y >= 0 && y < shared->dim_y &&
           z >= 0 && z < shared->dim_z;
}


//////////////////////////////////////////////////////////
//check whether a cell is accessible for robot movement
int map_is_accessible(const SharedData* shared, int x, int y, int z) {
    if (!map_is_valid_cell(shared, x, y, z)) return 0;
    int cell = shared->grid[x][y][z];
    return cell != CELL_DEBRIS;
}


//////////////////////////////////////////////////////////
//check whether the cell lies on the boundary of the map
int map_is_boundary(const SharedData* shared, int x, int y, int z) {
    return x == 0 || x == shared->dim_x - 1 ||
           y == 0 || y == shared->dim_y - 1 ||
           z == 0 || z == shared->dim_z - 1;
}


//////////////////////////////////////////////////////////
//return the risk value of a cell
int map_get_risk(const SharedData* shared, int x, int y, int z) {
    if (!map_is_valid_cell(shared, x, y, z)) return 100;
    return shared->risk_map[x][y][z];
}


//////////////////////////////////////////////////////////
//compute the Euclidean distance between two 3D points
double map_euclidean_distance(Point3D a, Point3D b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}


//////////////////////////////////////////////////////////
//select a random available entry point
Point3D map_get_random_entry(const SharedData* shared) {
    //if no entry points exist, return origin
    if (shared->num_entry_points == 0) {
        return point3d_create(0, 0, 0);
    }
    
    int available[MAX_ENTRY_POINTS];
    int count = 0;
    
    //collect all available entry points
    for (int i = 0; i < shared->num_entry_points; i++) {
        if (shared->entry_points[i].is_available) {
            available[count++] = i;
        }
    }

    //if none are available, return the first entry point
    if (count == 0) {
        return shared->entry_points[0].position;
    }
    
    //select a random available entry point
    int idx = available[utils_random_int(0, count - 1)];
    return shared->entry_points[idx].position;
}


//////////////////////////////////////////////////////////
void map_print(const SharedData* shared) {
    printf("3D MAP VIEW");
    
    for (int z = 0; z < shared->dim_z; z++) {
        map_print_layer(shared, z);
    }
}


//////////////////////////////////////////////////////////
void map_print_layer(const SharedData* shared, int z) {
    printf("\n%s=== Layer Z = %d ===%s\n", COLOR_CYAN, z, COLOR_RESET);
    
    printf("   ");
    for (int x = 0; x < shared->dim_x; x++) {
        printf("%d", x % 10);
    }
    printf("\n");
    
    printf("  +");
    for (int x = 0; x < shared->dim_x; x++) printf("-");
    printf("+\n");
    
    for (int y = 0; y < shared->dim_y; y++) {
        printf("%2d|", y);
        for (int x = 0; x < shared->dim_x; x++) {
            int cell = shared->grid[x][y][z];
            char symbol;
            const char* color;
            
            switch (cell) {
                case CELL_EMPTY:    symbol = '.'; color = COLOR_WHITE;   break;
                case CELL_DEBRIS:   symbol = '#'; color = COLOR_RED;     break;
                case CELL_SURVIVOR: symbol = 'S'; color = COLOR_GREEN;   break;
                case CELL_ENTRY:    symbol = 'E'; color = COLOR_YELLOW;  break;
                case CELL_RESCUED:  symbol = 'R'; color = COLOR_CYAN;    break;
                case CELL_ROBOT:    symbol = '@'; color = COLOR_MAGENTA; break;
                default:            symbol = '?'; color = COLOR_WHITE;   break;
            }
            
            printf("%s%c%s", color, symbol, COLOR_RESET);
        }
        printf("|\n");
    }
    
    printf("  +");
    for (int x = 0; x < shared->dim_x; x++) printf("-");
    printf("+\n");
}