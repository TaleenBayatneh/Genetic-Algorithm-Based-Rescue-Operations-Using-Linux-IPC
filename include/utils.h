#ifndef UTILS_H
#define UTILS_H

#include "common.h"

/* Random */
void utils_seed_random(unsigned int seed);
int utils_random_int(int min, int max);
double utils_random_double(double min, double max);
int utils_random_choice(double probability);

/* Point3D */
Point3D point3d_create(int x, int y, int z);
int point3d_equals(Point3D a, Point3D b);

/* Direction */
Point3D utils_apply_direction(Point3D pos, int direction);

/* Output */
void utils_print_separator(char c, int length);

/* Colors */
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_WHITE   "\033[37m"
#define COLOR_BOLD    "\033[1m"

/* Logging */
typedef enum {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR
} LogLevel;

extern LogLevel g_log_level;

void utils_log(LogLevel level, const char* format, ...);

#endif /* UTILS_H */
