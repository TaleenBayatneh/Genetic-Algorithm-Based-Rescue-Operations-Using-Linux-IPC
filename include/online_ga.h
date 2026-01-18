#ifndef ONLINE_GA_H
#define ONLINE_GA_H

#include "common.h"

void online_ga_init(LocalPopulation* pop, Robot* robot, SharedData* shared, 
                    const Config* config);

void online_ga_iterate(LocalPopulation* pop, Robot* robot, SharedData* shared,
                       const Config* config);

int online_ga_generate_exploration_path(Chromosome* chr, Robot* robot,
                                        SharedData* shared, const Config* config);

int online_ga_generate_rescue_path(Chromosome* chr, Robot* robot,
                                   Point3D target, SharedData* shared,
                                   const Config* config);

double online_ga_fitness(Chromosome* chr, Robot* robot, SharedData* shared,
                         const Config* config);

Chromosome* online_ga_tournament(LocalPopulation* pop, const Config* config);

void online_ga_crossover(const Chromosome* p1, const Chromosome* p2,
                         Chromosome* c1, Chromosome* c2,
                         Robot* robot, SharedData* shared, const Config* config);

void online_ga_mutate(Chromosome* chr, Robot* robot, SharedData* shared,
                      const Config* config);

int online_ga_validate_path(Chromosome* chr, Robot* robot, SharedData* shared);

Chromosome* online_ga_get_best(LocalPopulation* pop);

int online_ga_needs_replan(Robot* robot, SharedData* shared, const Config* config);

void online_ga_plan(Robot* robot, SharedData* shared, const Config* config);

#endif /* ONLINE_GA_H */
