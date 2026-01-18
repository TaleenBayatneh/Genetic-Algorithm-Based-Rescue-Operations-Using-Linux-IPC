#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <sys/sem.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <float.h>

///////////////////////// CONSTANTS ==================== /////////////////////

// Maximum map dimensions 
#define MAX_X 50
#define MAX_Y 50
#define MAX_Z 10

// Maximum system sizes 
#define MAX_ROBOTS 20
#define MAX_SURVIVORS 100
#define MAX_PATH_LENGTH 500

// Genetic Algorithm limits 
#define MAX_POPULATION 100
#define MAX_GENERATIONS 50

#define MAX_ENTRY_POINTS 100
#define MAX_LINE_LENGTH 256
#define MAX_RESCUES_PER_ROBOT 50  //Maximum rescues per robot 

////////////////////////// CELL TYPES  //////////////////////////////

#define CELL_UNKNOWN   -1  // Cell not yet explored
#define CELL_EMPTY      0  // Free cell
#define CELL_DEBRIS     1  // Obstacle or debris
#define CELL_SURVIVOR   2  // Survivor present
#define CELL_ENTRY      3  // Entry point (base)
#define CELL_RESCUED    4  // Survivor rescued
#define CELL_ROBOT      5  // Robot position

// Number of movement directions in 3D 
#define NUM_DIRECTIONS 26

//////////////////////////// MESSAGE TYPES ////////////////////////////////

#define MSG_CELL_DISCOVERED   1  // New cell discovered
#define MSG_SURVIVOR_RESCUED  3  // Survivor rescued
#define MSG_TERMINATE         5  // Terminate simulation

////////////////////////// SEMAPHORE INDICES /////////////////////////////

#define SEM_MUTEX       0  // General mutex
#define SEM_MAP_ACCESS  1  // Protect map access
#define SEM_SURVIVOR    2  // Protect survivor data
#define NUM_SEMS        3  // Total semaphores

/////////////////////////// ROBOT STATES /////////////////////////////

#define ROBOT_STATE_IDLE         0  // Waiting at base
#define ROBOT_STATE_EXPLORING    1  // Exploring unknown areas
#define ROBOT_STATE_APPROACHING  2  // Moving toward survivor
#define ROBOT_STATE_RESCUING     3  // Performing rescue
#define ROBOT_STATE_RETURNING    4  // Returning to base
#define ROBOT_STATE_REFILLING    5  // Refilling supplies
#define ROBOT_STATE_FINISHED     6  // Mission completed

/////////////////////////////////////// STRUCTURES ////////////////////////////////////////

// Represents a 3D coordinate 
typedef struct {
    int x;  // X coordinate (width)
    int y;  // Y coordinate (height)
    int z;  // Z coordinate (depth/floor)
} Point3D;

// Information about detected survivors
typedef struct {
    int id;                    // Survivor ID
    Point3D estimated_pos;     // Estimated position
    int heat_signal;           // Heat sensor strength
    int co2_signal;            // CO2 sensor strength
    int confirmed;             // 1 if confirmed
    int assigned_robot;        // Robot assigned to rescue
    double detection_time;     // Detection timestamp
} DetectedSurvivor;

// Confirmed  survivor information 
typedef struct {
    int id;                    // Unique survivor ID
    Point3D position;          // Exact position
    int is_alive;              // 1 if alive
    int is_rescued;            // 1 if rescued
    int is_detected;           // 1 if detected
    int rescued_by_robot;      // Robot ID that rescued
    double rescue_time;        // Rescue duration
    int heat_level;            // Heat emission level
    int co2_level;             // CO2 emission level
    int priority;              // Rescue priority
} Survivor;

// One movement step in GA 
typedef struct {
    Point3D position;  // The robot’s position after applying this movement step
    int direction;     // Direction index (0–25)
} Gene;

// A path solution used by GA 
typedef struct {
    Gene genes[MAX_PATH_LENGTH]; // Movement sequence
    int length;                  // Path length
    double fitness;              // Fitness score
    int target_type;             // 0=explore, 1=rescue, 2=return
    int target_id;               // Target survivor ID
    Point3D target_pos;          // Target position
    double path_length;          // Total distance
    double exploration_value;    // Exploration score
    int is_valid;                // 1 if valid
    Point3D start_pos;           // Start position
    Point3D end_pos;             // End position
} Chromosome;

// Genetic Algorithm population 
typedef struct {
    Chromosome individuals[MAX_POPULATION]; // Population
    int size;                               // Population size
    int generation;                         // Current generation
    double best_fitness;                    // Best fitness
    double avg_fitness;                     // Average fitness
} LocalPopulation;

// Robot runtime state 
typedef struct {
    int id;                     // Robot ID
    pid_t pid;                  // Process ID
    Point3D current_pos;        // Current position
    Point3D start_pos;          // Entry point
    Point3D base_pos;           // Base position

    int has_supplies;           // 1 if supplies available
    int current_survivor_id;    // Survivor being rescued

    int is_active;              // 1 if active
    int is_finished;            // 1 if finished

    int survivors_saved[MAX_RESCUES_PER_ROBOT]; // Rescued IDs
    int num_survivors_saved;    // Total rescues
    int total_trips;            // Trips to base

    int state;                  // Robot state
    Chromosome current_path;    // Current path
    int path_step;              // Current step index
    int needs_replan;           // 1 if replan required

    int heat_detected;          // Heat sensor triggered
    int co2_detected;           // CO2 sensor triggered
    Point3D heat_source;        // Heat source location
    Point3D co2_source;         // CO2 source location

    int cells_discovered;       // Cells explored
    int survivors_detected;     // Survivors detected

    double total_time;          // Active time
    double start_time;          // Start time
    int steps_taken;            // Steps moved

    int known_map[MAX_X][MAX_Y][MAX_Z];        // Known map
    int exploration_map[MAX_X][MAX_Y][MAX_Z]; // Exploration history
} Robot;

// Entry point structure 
typedef struct {
    Point3D position;  // Entry location
    int is_available;  // 1 if free
} EntryPoint;

//Simulation statistics 
typedef struct {
    int total_survivors;         // Total survivors
    int total_ga_runs;           // GA executions
    int total_replans;           // Replanning count
    int total_trips;             // Total robot trips
} Statistics;

// Shared memory structure 
typedef struct {
    int grid[MAX_X][MAX_Y][MAX_Z];         // Ground truth map
    int risk_map[MAX_X][MAX_Y][MAX_Z];     // Risk levels

    int discovered[MAX_X][MAX_Y][MAX_Z];  // Discovered map
    int discovered_by[MAX_X][MAX_Y][MAX_Z]; // Discovering robot

    int dim_x;  // Map width
    int dim_y;  // Map height
    int dim_z;  // Map depth

    Survivor survivors[MAX_SURVIVORS];     // Survivors list
    int num_survivors;
    int survivors_rescued;
    int survivors_detected;

    DetectedSurvivor detected_survivors[MAX_SURVIVORS];
    int num_detected;

    Robot robots[MAX_ROBOTS];
    int num_robots;
    int robots_finished;

    EntryPoint entry_points[MAX_ENTRY_POINTS];
    int num_entry_points;

    int simulation_running;       // 1 if running
    int simulation_paused;        // 1 if paused
    double simulation_start_time;
    double simulation_current_time;

    Statistics stats;

    int ready_flags[MAX_ROBOTS];  // Robot sync flags
    int terminate_flag;           // Termination signal

    int total_cells;              // Total cells
    int total_discovered;         // Discovered cells
} SharedData;

// IPC message structure 
typedef struct {
    long mtype;              // Message type
    int sender_id;           // Sender robot ID
    int receiver_id;         // Receiver ID
    int message_type;        // Message category
    int data_int;            // Integer payload
    Point3D location;        // Location data
    int cell_type;           // Cell information
    double timestamp;        // Time sent
    char data[64];           // Extra data
} IPCMessage;

// configuration parameters 
typedef struct {
    int map_width;
    int map_height;
    int map_depth;
    double debris_density;
    int num_survivors;
    int num_robots;

    int local_population_size;
    int local_max_generations;
    double mutation_rate;
    double crossover_rate;
    double elitism_rate;
    int replan_interval;

    double weight_exploration;
    double weight_survivor;
    double weight_distance;
    double weight_unknown;

    double robot_speed;
    double rescue_time;
    double refill_time;
    double max_simulation_time;

    int visual_range;
    int heat_sensor_range;
    int co2_sensor_range;

    double debris_risk;
    unsigned int random_seed;
} Config;

// Direction vectors 
extern const int DIR_X[NUM_DIRECTIONS];
extern const int DIR_Y[NUM_DIRECTIONS];
extern const int DIR_Z[NUM_DIRECTIONS];
extern const char* DIR_NAMES[NUM_DIRECTIONS];

void error_exit(const char* msg);
double get_current_time(void);

//Returns elapsed time since start_time
double get_elapsed_time(double start_time);

#endif