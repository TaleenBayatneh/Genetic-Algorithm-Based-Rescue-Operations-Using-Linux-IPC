#include "utils.h"
#include <stdarg.h>

/* Global log level controls which messages are printed by `utils_log()`.
    Can be set to `LOG_DEBUG`, `LOG_INFO`, etc., to increase/decrease verbosity. */
LogLevel g_log_level = LOG_INFO;

/* Direction vectors for the 26 neighbor moves in 3D.
    Used by path generation and movement helpers (e.g. `utils_apply_direction`).
    Indexes correspond to names in `DIR_NAMES`. */
const int DIR_X[NUM_DIRECTIONS] = {
    1, -1, 0, 0, 1, 1, -1, -1,
    0, 1, -1, 0, 0, 1, 1, -1, -1,
    0, 1, -1, 0, 0, 1, 1, -1, -1
};

/* Y component for the 26 directions. Keep ordering consistent with DIR_X/DIR_Z. */
const int DIR_Y[NUM_DIRECTIONS] = {
    0, 0, 1, -1, 1, -1, 1, -1,
    0, 0, 0, 1, -1, 1, -1, 1, -1,
    0, 0, 0, 1, -1, 1, -1, 1, -1
};

/* Z component for the 26 directions. Positive = up, negative = down. */
const int DIR_Z[NUM_DIRECTIONS] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1
};

/* Human-readable 3D direction names aligned with the DIR_* arrays. Useful
    for logging or debugging movement choices. */
const char* DIR_NAMES[NUM_DIRECTIONS] = {
    "E", "W", "N", "S", "NE", "SE", "NW", "SW",
    "UP", "UP-E", "UP-W", "UP-N", "UP-S", "UP-NE", "UP-SE", "UP-NW", "UP-SW",
    "DN", "DN-E", "DN-W", "DN-N", "DN-S", "DN-NE", "DN-SE", "DN-NW", "DN-SW"
};

/* error_exit: print an error message (with errno) and terminate.
   Use when a fatal system error occurs (IPC setup, memory alloc, etc.). */
void error_exit(const char* msg) {
    fprintf(stderr, "%s[ERROR]%s %s: %s\n", COLOR_RED, COLOR_RESET, msg, strerror(errno));
    exit(EXIT_FAILURE);
}

/* get_current_time: returns wall-clock time in seconds (fractional).
   Used for timing, stats, and simulation timestamps. */
double get_current_time(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

/* get_elapsed_time: convenience wrapper returning seconds since `start_time`. */
double get_elapsed_time(double start_time) {
    return get_current_time() - start_time;
}

/* utils_seed_random: initialize the PRNG for reproducible runs.
   If `seed==0` a time+pid-derived seed is used. Call once at startup. */
void utils_seed_random(unsigned int seed) {
    if (seed == 0) {
        seed = (unsigned int)time(NULL) ^ (unsigned int)getpid();
    }
    srand(seed);
}

/* utils_random_int: inclusive random integer in [min,max].
   Returns `min` when range is invalid. Used throughout for randomized choices. */
int utils_random_int(int min, int max) {
    if (min >= max) return min;
    return min + rand() % (max - min + 1);
}

/* utils_random_double: uniform random double in [min,max). */
double utils_random_double(double min, double max) {
    return min + (double)rand() / RAND_MAX * (max - min);
}

/* utils_random_choice: boolean draw with given probability (0..1). */
int utils_random_choice(double probability) {
    return utils_random_double(0, 1) < probability;
}

/* point3d_create: small helper to construct `Point3D` values. */
Point3D point3d_create(int x, int y, int z) {
    Point3D p = {x, y, z};
    return p;
}

/* point3d_equals: compare two 3D points for exact equality. */
int point3d_equals(Point3D a, Point3D b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

/* utils_apply_direction: return the position reached from `pos` by moving one
   step in `direction`. Bounds-checks direction index and leaves `pos` unchanged
   when invalid. Used by planners and movement simulation. */
Point3D utils_apply_direction(Point3D pos, int direction) {
    if (direction < 0 || direction >= NUM_DIRECTIONS) return pos;
    return point3d_create(
        pos.x + DIR_X[direction],
        pos.y + DIR_Y[direction],
        pos.z + DIR_Z[direction]
    );
}

/* utils_print_separator: convenience for printing visual separators to stdout. */
void utils_print_separator(char c, int length) {
    for (int i = 0; i < length; i++) putchar(c);
    putchar('\n');
}

/* utils_log: formatted logging with level filtering and colorized output.
   Call this for debug/info/warning/error messages. It honors `g_log_level`.
   Note: does not append a newline automatically — supply `\n` in format when
   needed. */
void utils_log(LogLevel level, const char* format, ...) {
    if (level < g_log_level) return;
    
    const char* level_str;
    const char* color;
    
    switch (level) {
        case LOG_DEBUG:   level_str = "DEBUG";   color = COLOR_CYAN;    break;
        case LOG_INFO:    level_str = "INFO";    color = COLOR_GREEN;   break;
        case LOG_WARNING: level_str = "WARNING"; color = COLOR_YELLOW;  break;
        case LOG_ERROR:   level_str = "ERROR";   color = COLOR_RED;     break;
        default:          level_str = "???";     color = COLOR_WHITE;   break;
    }
    
    va_list args;
    va_start(args, format);
    fprintf(stderr, " \n");
    fprintf(stderr, "%s[%s]%s ", color, level_str, COLOR_RESET);
    vfprintf(stderr, format, args);
    va_end(args);
}
