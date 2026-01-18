#include "common.h"
#include "config.h"
#include "map.h"
#include "online_ga.h"
#include "discovery.h"
#include "robot.h"
#include "ipc.h"
#include "utils.h"
#include "astar.h"

/////////////////////////////// GLOBAL VARIABLES //////////////////////////////////

//IPC resources used by simulation
static IPCResources g_ipc = {-1, -1, -1, NULL};

//Number of robot child processes created 
static int g_children_spawned = 0;

// cleanup_handler
void cleanup_handler(int sig) {
    (void)sig;  

    printf("\n\nReceived interrupt. Cleaning up...\n");

    //Notify all robots to terminate 
    if (g_ipc.shared_data) {
        g_ipc.shared_data->terminate_flag = 1;
    }

    // Wait for all child processes to exit 
    if (g_children_spawned > 0) {
        for (int i = 0; i < g_children_spawned; i++) {
            wait(NULL);
        }
    }

    // Release shared memory, semaphores, and message queues 
    ipc_cleanup(&g_ipc);

    // Exit the program 
    exit(0);
}

// Displays the map as discovered by robots during exploration,Unknown cells are shown as '?'.
void print_discovered_map(SharedData* shared) {
    printf("\n%s=== DISCOVERED MAP ===%s\n", COLOR_CYAN, COLOR_RESET);

    // Loop through map layers (Z axis) 
    for (int z = 0; z < shared->dim_z; z++) {
        printf("Layer Z=%d:\n   ", z);

        // Print X-axis indices 
        for (int x = 0; x < shared->dim_x; x++)
            printf("%d", x % 10);
        printf("\n");

        // Print each row (Y axis) 
        for (int y = 0; y < shared->dim_y; y++) {
            printf("%2d|", y);

            // Print each cell (X axis) 
            for (int x = 0; x < shared->dim_x; x++) {
                int cell = shared->discovered[x][y][z];
                char c = '?';  // Default for unknown cell

                if (cell != CELL_UNKNOWN) {
                    switch (cell) {
                        case CELL_EMPTY:    c = '.'; break;
                        case CELL_DEBRIS:   c = '#'; break;
                        case CELL_SURVIVOR: c = 'S'; break;
                        case CELL_ENTRY:    c = 'E'; break;
                        case CELL_RESCUED:  c = 'R'; break;
                    }
                }
                printf("%c", c);
            }
            printf("|\n");
        }
    }
}

void print_final_results(SharedData* shared, const Config* config, double total_time) {
    (void)config;  

    printf("\n");
    printf("             GA RESCUE OPERATION - FINAL REPORT                    \n");

    //DISCOVERY STATS 
    printf("\n DISCOVERY STATISTICS \n");
    printf(" Total Cells:       %-5d\n", shared->total_cells);
    printf(" Cells Discovered:  %-5d (%.1f%%)\n",
           shared->total_discovered, discovery_get_progress(shared));
    printf(" Survivors Detected: %d / %d\n",
           shared->survivors_detected, shared->num_survivors);


    // SURVIVOR DETAILS 
    printf("\n============================= SURVIVOR RESCUE DETAILS =============================\n");

int rescued = 0;

for (int s = 0; s < shared->num_survivors; s++) {
    Survivor* surv = &shared->survivors[s];

    // Determine survivor status
    const char* status = surv->is_rescued ? "RESCUED" :(surv->is_detected ? "DETECTED" : "UNKNOWN");

    printf("\nSurvivor #%d\n", surv->id);
    printf("----------------------------\n");
    printf("Position        : (%d, %d, %d)\n",
           surv->position.x,
           surv->position.y,
           surv->position.z);
    printf("Status          : %s\n", status);
    printf("Rescued by Robot: %d\n", surv->rescued_by_robot);
    printf("Rescue Time     : %.2f seconds\n", surv->rescue_time);
    printf("Heat Level      : %d\n", surv->heat_level);
    printf("CO2 Level       : %d\n", surv->co2_level);
    printf("----------------------------\n");


    if (surv->is_rescued)
        rescued++;
}

printf("TOTAL: %d / %d survivors rescued (%.1f%%)\n",
       rescued,
       shared->num_survivors,
       100.0 * rescued / shared->num_survivors);


    printf("\n============================= ROBOT PERFORMANCE =====================================\n");

int total_trips = 0;

for (int r = 0; r < shared->num_robots; r++) {
    Robot* robot = &shared->robots[r];

    printf("\nRobot #%d\n", robot->id);
    printf("----------------------------\n");
    printf("Total Trips        : %d\n", robot->total_trips);
    printf("Survivors Rescued  : %d\n", robot->num_survivors_saved);
    printf("Steps Taken        : %d\n", robot->steps_taken);
    printf("Cells Discovered  : %d\n", robot->cells_discovered);
    printf("Total Time        : %.2f seconds\n", robot->total_time);
    printf("Final State       : %s\n", robot_state_name(robot->state));

    total_trips += robot->total_trips;

    // Print survivors rescued by this robot
    if (robot->num_survivors_saved > 0) {
        printf("Rescued Survivors : ");
        for (int i = 0; i < robot->num_survivors_saved; i++) {
            printf("%d", robot->survivors_saved[i]);
            if (i < robot->num_survivors_saved - 1)
                printf(", ");
        }
        printf("\n");
    } else {
        printf("Rescued Survivors : None\n");
    }
     printf("----------------------------\n");

}

printf("TOTAL TRIPS (All Robots): %d\n", total_trips);

printf("\n======================================\n");

    printf("\n STATISTICS SUMMARY \n");
    printf(" Total Simulation Time: %.2f seconds\n", total_time);
    printf(" Total Trips Made:      %d\n", total_trips);
    printf(" Total GA Replans:      %d\n", shared->stats.total_replans);
    printf(" Rescue Success Rate:   %.1f%%\n",
           100.0 * rescued / shared->num_survivors);
    printf(" Map Explored:          %.1f%%\n",
           discovery_get_progress(shared));
    printf(" Avg Survivors/Robot:   %.1f\n",
           (double)rescued / shared->num_robots);
    printf(" Avg Trips/Robot:       %.1f\n",
           (double)total_trips / shared->num_robots);
}

int main(int argc, char* argv[]) {
    Config config;
    double start_time, end_time;
    AStarResults astar_full_results;
    
    signal(SIGINT, cleanup_handler);
    signal(SIGTERM, cleanup_handler);
    
    printf("\n");
    printf("────────────────────────────────────────────────────────────────────────\n");
    printf("          GENETIC ALGORITHM RESCUE ROBOT SIMULATOR               \n");
    printf("          Discovery + Multiple Trips + A* Comparison             \n");
    printf("────────────────────────────────────────────────────────────────────────\n");
    
    const char* config_file;

    if (argc > 1) {
        config_file = argv[1];          
    } else {
        config_file = "config.txt"; 
    }

    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        printf("Usage: %s [config_file]\n", argv[0]);
        printf("\nRobots make MULTIPLE TRIPS to rescue all survivors.\n");
        printf("After each rescue, robot returns to base, refills, and goes again.\n");
        return 0;
    }
    
    config_load(&config, config_file);
    
    if (!config_validate(&config)) {
        utils_log(LOG_ERROR, "Invalid configuration");
        return 1;
    }
    
    config_print(&config);
    
    utils_seed_random(config.random_seed);
    
    if (ipc_init(&g_ipc, &config) < 0) {
        utils_log(LOG_ERROR, "Failed to initialize IPC");
        return 1;
    }
    
    SharedData* shared = g_ipc.shared_data;
    
    map_init(shared, &config);
    map_generate(shared, &config);
    discovery_init(shared, &config);
    robot_init_all(shared, &config);
    
    printf("\n%sGround Truth Map:%s\n", COLOR_BOLD, COLOR_RESET);
    map_print(shared);
    
    printf("\nSurvivors: %d | Robots: %d | Entry points: %d\n",shared->num_survivors, shared->num_robots, shared->num_entry_points);
    printf("Each robot will make MULTIPLE TRIPS until all survivors rescued.\n");
    
    // A* Baseline 
    printf("────────────────────────────────────────────────────────────────────────\n");
    printf("\n%s A* BASELINE%s\n\n", COLOR_BOLD, COLOR_RESET);
    printf("────────────────────────────────────────────\n");
    astar_full_results = astar_run_full_knowledge(shared, &config);
    astar_print_results(&astar_full_results);
    
    // GA Simulation 
    printf("────────────────────────────────────────────────────────────────────────\n");
    printf("\n%s GA SIMULATION (Multiple Trips):%s\n\n", COLOR_BOLD, COLOR_RESET);
    printf("────────────────────────────────────────────\n");
    start_time = get_current_time();
    shared->simulation_running = 1;
    shared->simulation_start_time = start_time;
    
    printf("initializing %d robot processes...\n", shared->num_robots);
    printf("Robots will return to base after each rescue for new supplies.\n");

    pid_t pids[MAX_ROBOTS];
    for (int i = 0; i < shared->num_robots; i++) {
        pids[i] = fork();
        
        if (pids[i] < 0) {
            error_exit("Fork failed");
        } else if (pids[i] == 0) {
            robot_process_main(i, shared, &g_ipc, &config);
            exit(0);
        }
        
        shared->robots[i].pid = pids[i];
        g_children_spawned++;
    }
    
    int last_rescued = 0;
    int last_trips = 0;
    
    while (shared->robots_finished < shared->num_robots) {
        
        // Calculate total trips 
        int total_trips = 0;
        for (int r = 0; r < shared->num_robots; r++) {
            total_trips += shared->robots[r].total_trips;
        }
        
        if (shared->survivors_rescued != last_rescued || total_trips != last_trips) {
            printf("\rLatest update: Elapsed Time:[%6.1fs] | Explored: %3.0f%% | Rescued: %d/%d | Trips: %d \n",get_elapsed_time(start_time),discovery_get_progress(shared),shared->survivors_rescued, shared->num_survivors,total_trips);
            fflush(stdout);
            
            last_rescued = shared->survivors_rescued;
            last_trips = total_trips;
        }
        
        if (get_elapsed_time(start_time) > config.max_simulation_time) {
            printf("\n\n%sTimeout reached.%s\n", COLOR_YELLOW, COLOR_RESET);
            shared->terminate_flag = 1;
            break;
        }
        
        if (shared->survivors_rescued >= shared->num_survivors) {
            printf("\n\n%sAll %d survivors rescued!%s\n",COLOR_GREEN, shared->num_survivors, COLOR_RESET);
            shared->terminate_flag = 1;
            break;
        }
    }
    
    printf("\nWaiting for robot processes to finish...\n");
    for (int i = 0; i < g_children_spawned; i++) {
        int status;
        waitpid(pids[i], &status, 0);
    }
    g_children_spawned = 0;
    
    shared->simulation_running = 0;
    end_time = get_current_time();
    double ga_total_time = end_time - start_time;
    
    // Results 
    printf("\n────────────────────────────────────────────────────────────────────────\n");
    printf("\n%s RESULTS:%s\n\n", COLOR_BOLD, COLOR_RESET);
    printf("────────────────────────────────────────────");
    printf("\n%sFinal Ground Truth Map:%s\n", COLOR_BOLD, COLOR_RESET);
    map_print(shared);
    
    print_discovered_map(shared);
    print_final_results(shared, &config, ga_total_time);
    
    // Comparison 
    printf("────────────────────────────────────────────────────────────────────────\n");   
    printf("\n%s COMPARISON:%s\n\n", COLOR_BOLD, COLOR_RESET);
    printf("────────────────────────────────────────────\n");
    astar_compare_with_online_ga(shared, &astar_full_results, ga_total_time, &config);
    
    ipc_cleanup(&g_ipc);
    
    printf("\n%s Simulation Completed %s\n\n", COLOR_GREEN, COLOR_RESET);
    
    return 0;
}
