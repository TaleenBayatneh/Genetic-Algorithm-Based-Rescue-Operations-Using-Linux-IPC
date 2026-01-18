#ifndef ASTAR_H
#define ASTAR_H

#include "common.h"

/* A* node structure */
typedef struct AStarNode {
    Point3D pos;
    double g_cost;
    double h_cost;
    double f_cost;
    struct AStarNode* parent;
    int direction;
} AStarNode;

/* Priority queue for A* */
typedef struct {
    AStarNode* nodes[MAX_PATH_LENGTH * 10];
    int size;
} PriorityQueue;

/* A* Results for comparison */
typedef struct {
    double total_path_length;
    double total_time;
    int survivors_rescued;
    int paths_found;
    double avg_path_length;
} AStarResults;

/* Find path from start to goal using A* (uses full map knowledge) */
int astar_find_path(const SharedData* shared, Point3D start, Point3D goal,
                    Chromosome* result_path, const Config* config);

/* Find path using only discovered cells (for fair comparison) */
int astar_find_path_discovered(const SharedData* shared, Point3D start, Point3D goal,
                               Chromosome* result_path, const Config* config);

/* Find path to nearest survivor */
int astar_find_nearest_survivor(const SharedData* shared, Point3D start,
                                Chromosome* result_path, int* survivor_id,
                                const Config* config);


/* Run A* single robot simulation (with full knowledge - baseline) */
AStarResults astar_run_full_knowledge(SharedData* shared, const Config* config);

/* Compare Online GA results with A* results */
void astar_compare_with_online_ga(SharedData* shared, const AStarResults* astar_results,
                                  double ga_total_time, const Config* config);

/* Print A* results */
void astar_print_results(const AStarResults* results);

#endif /* ASTAR_H */
