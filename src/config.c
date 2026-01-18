#include "config.h"
#include "utils.h"

// function to use the defualt values 
void config_init_defaults(Config* config) {
    if (!config) return;
    
    // Map parameters 
    config->map_width = 20;//map width
    config->map_height = 20;//map hight
    config->map_depth = 2; // number of floors
    config->debris_density = 0.20; // debris ratio
    config->num_survivors = 8; // number of survivors
    config->num_robots = 3; // number of robots
        
    // GA parameters 
    config->local_population_size = 25;//Number of individuals in each generation
    config->local_max_generations = 15;//Maximum number of generations
    config->mutation_rate = 0.20; // Probability of mutating a gene in an individual
    config->crossover_rate = 0.70; // Probability of mutating a gene in an individual
    config->elitism_rate = 0.15; // Percentage of top individuals carried unchanged to next generation
    config->replan_interval = 10;
    
        
    // Fitness weights: used in fitness function 
    config->weight_exploration = 5.0;
    config->weight_survivor = 100.0; //Important to find more survivors
    config->weight_distance = 2.0;
    config->weight_unknown = 8.0;
    
    // Simulation parameters 
    config->robot_speed = 3.0; // How fast the robot moves
    config->rescue_time = 2.0; // time to rescue one survivor
    config->refill_time = 1.0; //Time to refill at base 
    config->max_simulation_time = 180.0;  // Maximum time for the simulation to run
    
    config->visual_range = 3;
    config->heat_sensor_range = 6;
    config->co2_sensor_range = 5;
    
    // Risk parameters 
    config->debris_risk = 0.1;// Chance of getting hurt by debris
    
    
    // Random seed for generating random numbers (0 = different results each run)
    config->random_seed = 0;}

// Function to remove leading and trailing whitespace from a string
static char* trim_string(char* str) {
        if (!str) return NULL;
        while (*str == ' ' || *str == '\t') str++;
        if (*str == '\0') return str;
        char* end = str + strlen(str) - 1;
        while (end > str && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) {
            *end-- = '\0';
        }
        return str;
}

// function to load the config from file 
int config_load(Config* config, const char* filename) {
    FILE* file;
    char line[MAX_LINE_LENGTH];
    char key[128], value[128];
    
    if (!config || !filename) return -1;
    
    // Initialize with defaults first 
    config_init_defaults(config);
    
    file = fopen(filename, "r");
    if (!file) {
        utils_log(LOG_WARNING, "Config file '%s' not found, using defaults", filename);
        return 0;
    }
    
    utils_log(LOG_INFO, "Loading configuration from '%s'", filename);
    
    while (fgets(line, sizeof(line), file)) {
         // Skip empty lines and comments 
        char* trimmed = trim_string(line);
        if (strlen(trimmed) == 0 || trimmed[0] == '#' || trimmed[0] == ';') {
            continue;
        }
        // Parse key=value pairs 
        if (sscanf(trimmed, "%127[^=]=%127s", key, value) == 2) {
            // Remove spaces from the key
            char* k = trim_string(key);
            char* v = trim_string(value);
            
            // Map parameters 
            if (strcmp(k, "map_width") == 0) config->map_width = atoi(v);
            else if (strcmp(k, "map_height") == 0) config->map_height = atoi(v);
            else if (strcmp(k, "map_depth") == 0) config->map_depth = atoi(v);
            else if (strcmp(k, "debris_density") == 0) config->debris_density = atof(v);
            else if (strcmp(k, "num_survivors") == 0) config->num_survivors = atoi(v);
            else if (strcmp(k, "num_robots") == 0) config->num_robots = atoi(v);
            // GA parameters 
            else if (strcmp(k, "local_population_size") == 0) config->local_population_size = atoi(v);
            else if (strcmp(k, "local_max_generations") == 0) config->local_max_generations = atoi(v);
            else if (strcmp(k, "mutation_rate") == 0) config->mutation_rate = atof(v);
            else if (strcmp(k, "crossover_rate") == 0) config->crossover_rate = atof(v);
            else if (strcmp(k, "elitism_rate") == 0) config->elitism_rate = atof(v);
            else if (strcmp(k, "replan_interval") == 0) config->replan_interval = atoi(v);
            // Fitness weights 
            else if (strcmp(k, "weight_exploration") == 0) config->weight_exploration = atof(v);
            else if (strcmp(k, "weight_survivor") == 0) config->weight_survivor = atof(v);
            else if (strcmp(k, "weight_distance") == 0) config->weight_distance = atof(v);
            else if (strcmp(k, "weight_unknown") == 0) config->weight_unknown = atof(v);
            // Simulation parameters
            else if (strcmp(k, "robot_speed") == 0) config->robot_speed = atof(v);
            else if (strcmp(k, "rescue_time") == 0) config->rescue_time = atof(v);
            else if (strcmp(k, "refill_time") == 0) config->refill_time = atof(v);
            else if (strcmp(k, "max_simulation_time") == 0) config->max_simulation_time = atof(v);

            else if (strcmp(k, "visual_range") == 0) config->visual_range = atoi(v);
            else if (strcmp(k, "heat_sensor_range") == 0) config->heat_sensor_range = atoi(v);
            else if (strcmp(k, "co2_sensor_range") == 0) config->co2_sensor_range = atoi(v);
            // Risk parameters 
            else if (strcmp(k, "debris_risk") == 0) config->debris_risk = atof(v);

            else if (strcmp(k, "random_seed") == 0) config->random_seed = (unsigned int)atoi(v);
        }
    }
    
    fclose(file);
    return 1;
}

// function to print the data to show in run 
void config_print(const Config* config) {
    if (!config) return;
    
    printf("\n%s=== GA CONFIGURATION ===%s\n", COLOR_CYAN, COLOR_RESET);
    printf("Map: %dx%dx%d, Debris: %.0f%%\n", 
           config->map_width, config->map_height, config->map_depth,
           config->debris_density * 100);
    printf("Robots: %d, Survivors: %d\n", config->num_robots, config->num_survivors);
    printf("Local GA: pop=%d, gen=%d, mut=%.2f, cross=%.2f\n",
           config->local_population_size, config->local_max_generations,
           config->mutation_rate, config->crossover_rate);
    printf("Sensors: visual=%d, heat=%d, co2=%d\n",
           config->visual_range, config->heat_sensor_range, config->co2_sensor_range);
    printf("Timing: speed=%.1f, rescue=%.1fs, refill=%.1fs, max=%.0fs\n",
           config->robot_speed, config->rescue_time, config->refill_time,
           config->max_simulation_time);
    utils_print_separator('=', 50);
}

//function to validate the input data 
int config_validate(const Config* config) {
    if (!config) return 0;

    int valid = 1;
    
    if (config->map_width <= 0 || config->map_width > MAX_X) {
        utils_log(LOG_ERROR, "Invalid map_width: %d", config->map_width);
        valid = 0;
    }
    if (config->map_height <= 0 || config->map_height > MAX_Y) {
        utils_log(LOG_ERROR, "Invalid map_height: %d", config->map_height);
        valid = 0;
    }
    if (config->map_depth <= 0 || config->map_depth > MAX_Z) {
        utils_log(LOG_ERROR, "Invalid map_depth: %d", config->map_depth);
        valid = 0;
    }
    if (config->num_robots <= 0 || config->num_robots > MAX_ROBOTS) {
        utils_log(LOG_ERROR, "Invalid num_robots: %d", config->num_robots);
        valid = 0;
    }
    if (config->num_survivors <= 0 || config->num_survivors > MAX_SURVIVORS) {
        utils_log(LOG_ERROR, "Invalid num_survivors: %d", config->num_survivors);
        valid = 0;
    }
    if (config->debris_density < 0 || config->debris_density > 0.8) {
        utils_log(LOG_ERROR, "Invalid debris_density: %.2f", config->debris_density);
        valid = 0;
    }
    
    return valid;
}
