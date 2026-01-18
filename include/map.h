#ifndef MAP_H
#define MAP_H

#include "common.h"

void map_init(SharedData* shared, const Config* config);
void map_generate(SharedData* shared, const Config* config);
void map_place_survivors(SharedData* shared, const Config* config);
void map_calculate_risks(SharedData* shared, const Config* config);
void map_find_entry_points(SharedData* shared);

int map_is_valid_cell(const SharedData* shared, int x, int y, int z);
int map_is_accessible(const SharedData* shared, int x, int y, int z);
int map_is_boundary(const SharedData* shared, int x, int y, int z);

int map_get_risk(const SharedData* shared, int x, int y, int z);

double map_euclidean_distance(Point3D a, Point3D b);

Point3D map_get_random_entry(const SharedData* shared);

void map_print(const SharedData* shared);
void map_print_layer(const SharedData* shared, int z);

#endif /* MAP_H */
