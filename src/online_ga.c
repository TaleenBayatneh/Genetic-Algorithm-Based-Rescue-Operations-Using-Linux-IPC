#include "online_ga.h"
#include "discovery.h"
#include "utils.h"
#include "map.h"

/*
 * compare_fitness
 * Role: sort chromosomes by fitness (highest first) for population ordering.
 * How: simple comparator used by `qsort` that compares `Chromosome.fitness`.
 */
static int compare_fitness(const void* a, const void* b) {
    const Chromosome* ca = (const Chromosome*)a;
    const Chromosome* cb = (const Chromosome*)b;
    if (cb->fitness > ca->fitness) return 1;
    if (cb->fitness < ca->fitness) return -1;
    return 0;
}

/*
 * online_ga_init
 * Role: build the initial local population for a robot's GA planner.
 * How: looks for an assigned survivor to bias rescue paths, then
 *       generates exploration or rescue chromosomes until the
 *       configured population size is reached; computes fitness
 *       and sorts the population.
 */
void online_ga_init(LocalPopulation* pop, Robot* robot, SharedData* shared,
                    const Config* config) {
    pop->size = 0;
    pop->generation = 0;
    pop->best_fitness = -DBL_MAX;
    pop->avg_fitness = 0;
    
    /* Determine if we have a target survivor */
    int target_survivor = -1;
    Point3D target_pos = robot->current_pos;
    
    for (int s = 0; s < shared->num_detected; s++) {
        DetectedSurvivor* ds = &shared->detected_survivors[s];
        if (!shared->survivors[ds->id].is_rescued && 
            (ds->assigned_robot == -1 || ds->assigned_robot == robot->id)) {
            target_survivor = ds->id;
            target_pos = ds->estimated_pos;
            break;
        }
    }
    
    /* Generate initial population */
    for (int i = 0; i < config->local_population_size && pop->size < config->local_population_size; i++) {
        Chromosome* chr = &pop->individuals[pop->size];
        memset(chr, 0, sizeof(Chromosome));
        
        int success = 0;
        if (target_survivor >= 0) {
            chr->target_type = 1;
            chr->target_id = target_survivor;
            success = online_ga_generate_rescue_path(chr, robot, target_pos, shared, config);
        } else {
            chr->target_type = 0;
            success = online_ga_generate_exploration_path(chr, robot, shared, config);
        }
        
        if (success && chr->is_valid && chr->length > 0) {
            online_ga_fitness(chr, robot, shared, config);
            pop->size++;
        }
    }
    
    /* If we couldn't generate any valid paths, create a simple one-step path */
    if (pop->size == 0) {
        Chromosome* chr = &pop->individuals[0];
        chr->genes[0].position = robot->current_pos;
        chr->length = 1;
        chr->is_valid = 1;
        chr->start_pos = robot->current_pos;
        chr->end_pos = robot->current_pos;
        chr->fitness = -100;
        pop->size = 1;
    }
    
    /* Sort by fitness */
    if (pop->size > 1) {
        qsort(pop->individuals, pop->size, sizeof(Chromosome), compare_fitness);
    }
    
    pop->best_fitness = pop->individuals[0].fitness;
}

/*
 * online_ga_iterate
 * Role: perform one GA generation to evolve the local population.
 * How: preserves elites, selects parents via tournaments, applies
 *       crossover and mutation, fills remaining slots with random
 *       exploration paths, then sorts and updates population stats.
 */
void online_ga_iterate(LocalPopulation* pop, Robot* robot, SharedData* shared,
                       const Config* config) {
    if (pop->size < 2) {
        online_ga_init(pop, robot, shared, config);
        return;
    }
    
    LocalPopulation new_pop;
    new_pop.size = 0;
    new_pop.generation = pop->generation + 1;
    
    /* Elitism: copy top-ranked individuals directly into next generation */
    int elite_count = (int)(pop->size * config->elitism_rate);
    if (elite_count < 1) elite_count = 1;
    if (elite_count > pop->size) elite_count = pop->size;
    
    for (int i = 0; i < elite_count; i++) {
        memcpy(&new_pop.individuals[new_pop.size++], &pop->individuals[i], sizeof(Chromosome));
    }
    
    /* Generate new individuals */
    int attempts = 0;
    int max_attempts = config->local_population_size * 3;
    
    while (new_pop.size < config->local_population_size && attempts < max_attempts) {
        attempts++;
        
        Chromosome* parent1 = online_ga_tournament(pop, config);
        Chromosome* parent2 = online_ga_tournament(pop, config);
        
        if (!parent1 || !parent2) continue;
        
        Chromosome child1, child2;
        online_ga_crossover(parent1, parent2, &child1, &child2, robot, shared, config);
        
        online_ga_mutate(&child1, robot, shared, config);
        online_ga_mutate(&child2, robot, shared, config);
        
        if (child1.is_valid && child1.length > 0 && new_pop.size < config->local_population_size) {
            online_ga_fitness(&child1, robot, shared, config);
            memcpy(&new_pop.individuals[new_pop.size++], &child1, sizeof(Chromosome));
        }
        if (child2.is_valid && child2.length > 0 && new_pop.size < config->local_population_size) {
            online_ga_fitness(&child2, robot, shared, config);
            memcpy(&new_pop.individuals[new_pop.size++], &child2, sizeof(Chromosome));
        }
    }
    
    /* Fill remaining slots with freshly generated exploration paths to
     * maintain genetic diversity in the population.
     */
    while (new_pop.size < config->local_population_size / 2) {
        Chromosome chr;
        memset(&chr, 0, sizeof(Chromosome));
        
        if (online_ga_generate_exploration_path(&chr, robot, shared, config)) {
            online_ga_fitness(&chr, robot, shared, config);
            memcpy(&new_pop.individuals[new_pop.size++], &chr, sizeof(Chromosome));
        } else {
            break;
        }
    }
    
    if (new_pop.size == 0) {
        return;  /* Keep old population */
    }
    
    /* Sort new population by fitness (best-first) */
    qsort(new_pop.individuals, new_pop.size, sizeof(Chromosome), compare_fitness);
    
    /* Update stats */
    double total = 0;
    for (int i = 0; i < new_pop.size; i++) {
        total += new_pop.individuals[i].fitness;
    }
    new_pop.avg_fitness = new_pop.size > 0 ? total / new_pop.size : 0;
    new_pop.best_fitness = new_pop.individuals[0].fitness;
    
    /* Atomic replacement of the population: overwrite old population with new */
    memcpy(pop, &new_pop, sizeof(LocalPopulation));
}

/*
 * online_ga_generate_exploration_path
 * Role: create a candidate path aimed at exploring unknown areas.
 * How: selects an exploration target via `discovery_find_exploration_target`,
 *       then greedily walks toward promising neighbors favoring unknown
 *       cells and frontiers, building a short path (bounded length).
 */
int online_ga_generate_exploration_path(Chromosome* chr, Robot* robot,
                                        SharedData* shared, const Config* config) {
    memset(chr, 0, sizeof(Chromosome));
    chr->target_type = 0;
    chr->start_pos = robot->current_pos;
    chr->is_valid = 0;
    
    /* Find exploration target */
    Point3D target = discovery_find_exploration_target(robot, shared, config);
    chr->target_pos = target;
    
    Point3D current = robot->current_pos;
    
    /* Use simple visited array
     * Note: declared `static` to avoid large stack allocation. The array is
     * zeroed each call with `memset`; 
     */
    static int visited[MAX_X][MAX_Y][MAX_Z];
    memset(visited, 0, sizeof(visited));
    visited[current.x][current.y][current.z] = 1;
    
    chr->genes[0].position = current;
    chr->genes[0].direction = -1;
    int steps = 1;
    int max_steps = 30;  /* Shorter paths for quicker planning */
    
    while (steps < max_steps) {
        /* Get valid neighbors */
        Point3D candidates[NUM_DIRECTIONS];
        int directions[NUM_DIRECTIONS];
        int num_candidates = 0;
        
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            Point3D next = utils_apply_direction(current, d);
            
            if (next.x < 0 || next.x >= shared->dim_x ||
                next.y < 0 || next.y >= shared->dim_y ||
                next.z < 0 || next.z >= shared->dim_z) {
                continue;
            }
            
            if (visited[next.x][next.y][next.z]) continue;
            
            /* Check what we know about this cell */
            int cell = robot->known_map[next.x][next.y][next.z];
            
            /* Can move to empty, entry, survivor, unknown cells */
            /* Cannot move to known debris */
            if (cell == CELL_DEBRIS) continue;
            
            candidates[num_candidates] = next;
            directions[num_candidates] = d;
            num_candidates++;
        }
        
        if (num_candidates == 0) break;
        
        /* Select best candidate */
        int best_idx = 0;
        double best_score = -DBL_MAX;
        
        for (int i = 0; i < num_candidates; i++) {
            Point3D next = candidates[i];
            double score = 0;
            
            /* Distance to target */
            double dist_to_target = map_euclidean_distance(next, target);
            score -= dist_to_target * 0.5;
            
            /* Exploration heuristic: reward unknown cells to favor frontiers.
             * The `weight_unknown * 3` multiplier biases greedy expansion
             * toward unseen areas when constructing exploration paths.
             */
            int cell = robot->known_map[next.x][next.y][next.z];
            if (cell == CELL_UNKNOWN) {
                score += config->weight_unknown * 3;
            }
            
            /* Count adjacent unknown cells */
            int unknown_adj = 0;
            for (int d = 0; d < 8; d++) {  /* Only check same-level neighbors for speed */
                int nx = next.x + DIR_X[d];
                int ny = next.y + DIR_Y[d];
                int nz = next.z;
                if (nx >= 0 && nx < shared->dim_x &&
                    ny >= 0 && ny < shared->dim_y &&
                    robot->known_map[nx][ny][nz] == CELL_UNKNOWN) {
                    unknown_adj++;
                }
            }
            score += unknown_adj * config->weight_exploration * 0.5;
            
            /* Small randomized perturbation to diversify candidate scores
             * and avoid deterministic ties or local optima.
             */
            score += utils_random_double(-2, 2);
            
            if (score > best_score) {
                best_score = score;
                best_idx = i;
            }
        }
        
        current = candidates[best_idx];
        visited[current.x][current.y][current.z] = 1;
        
        chr->genes[steps].position = current;
        chr->genes[steps].direction = directions[best_idx];
        steps++;
        
        /* Stop if reached target or unknown cell */
        if (point3d_equals(current, target)) break;
        int cell = robot->known_map[current.x][current.y][current.z];
        if (cell == CELL_UNKNOWN) break;
    }
    
    chr->length = steps;
    chr->end_pos = current;
    chr->is_valid = (steps > 0);
    
    /* Calculate path length */
    chr->path_length = 0;
    for (int i = 1; i < chr->length; i++) {
        chr->path_length += map_euclidean_distance(
            chr->genes[i-1].position,
            chr->genes[i].position);
    }
    
    return chr->is_valid;
}

/*
 * online_ga_generate_rescue_path
 * Role: construct a candidate path to reach a specific survivor (target).
 * How: greedily advances toward `target` using known free cells, avoids
 *       known debris and respects map bounds; records path and exploration value.
 */
int online_ga_generate_rescue_path(Chromosome* chr, Robot* robot,
                                   Point3D target, SharedData* shared,
                                   const Config* config) {
    memset(chr, 0, sizeof(Chromosome));
    chr->target_type = 1;
    chr->target_pos = target;
    chr->start_pos = robot->current_pos;
    chr->is_valid = 0;
    
    Point3D current = robot->current_pos;
    
    static int visited[MAX_X][MAX_Y][MAX_Z];
    memset(visited, 0, sizeof(visited));
    visited[current.x][current.y][current.z] = 1;
    
    chr->genes[0].position = current;
    chr->genes[0].direction = -1;
    int steps = 1;
    int max_steps = 100;
    
    while (steps < max_steps && !point3d_equals(current, target)) {
        Point3D candidates[NUM_DIRECTIONS];
        int directions[NUM_DIRECTIONS];
        int num_candidates = 0;
        
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            Point3D next = utils_apply_direction(current, d);
            
            if (next.x < 0 || next.x >= shared->dim_x ||
                next.y < 0 || next.y >= shared->dim_y ||
                next.z < 0 || next.z >= shared->dim_z) {
                continue;
            }
            
            if (visited[next.x][next.y][next.z]) continue;
            
            int cell = robot->known_map[next.x][next.y][next.z];
            if (cell == CELL_DEBRIS) continue;
            
            candidates[num_candidates] = next;
            directions[num_candidates] = d;
            num_candidates++;
        }
        
        if (num_candidates == 0) break;
        
        /* Greedy selection towards target */
        int best_idx = 0;
        double best_dist = DBL_MAX;
        
        for (int i = 0; i < num_candidates; i++) {
            double dist = map_euclidean_distance(candidates[i], target);
            
            /* Small random perturbation to avoid deterministic ties and help
             * escape local minima when greedily approaching a target.
             */
            dist += utils_random_double(-0.5, 0.5);
            
            /* Slight penalty for unknown cells */
            int cell = robot->known_map[candidates[i].x][candidates[i].y][candidates[i].z];
            if (cell == CELL_UNKNOWN) {
                dist += 0.5;
            }
            
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = i;
            }
        }
        
        current = candidates[best_idx];
        visited[current.x][current.y][current.z] = 1;
        
        chr->genes[steps].position = current;
        chr->genes[steps].direction = directions[best_idx];
        steps++;
    }
    
    chr->length = steps;
    chr->end_pos = current;
    chr->is_valid = (steps > 0);
    
    /* Calculate path length */
    chr->path_length = 0;
    chr->exploration_value = 0;
    
    for (int i = 1; i < chr->length; i++) {
        chr->path_length += map_euclidean_distance(
            chr->genes[i-1].position,
            chr->genes[i].position);
        
        int cell = robot->known_map[chr->genes[i].position.x]
                                   [chr->genes[i].position.y]
                                   [chr->genes[i].position.z];
        if (cell == CELL_UNKNOWN) {
            chr->exploration_value += 1;
        }
    }
    
    return chr->is_valid;
}

/*
 * online_ga_fitness
 * Role: score a chromosome to guide selection toward useful plans.
 * How: penalizes distance to target and path length, rewards exploring
 *       unknown cells and progress toward a survivor target.
 */
double online_ga_fitness(Chromosome* chr, Robot* robot, SharedData* shared,
                         const Config* config) {
    if (!chr->is_valid || chr->length == 0) {
        chr->fitness = -1000;
        return chr->fitness;
    }
    
    double fitness = 0;
    
    /* Distance to target */
    double dist_to_target = map_euclidean_distance(chr->end_pos, chr->target_pos);
    fitness -= dist_to_target * config->weight_distance;
    
    /* Path length penalty */
    fitness -= chr->path_length * 0.3;
    
    /* Exploration value */
    int unknown_cells = 0;
    for (int i = 0; i < chr->length; i++) {
        int cell = robot->known_map[chr->genes[i].position.x]
                                   [chr->genes[i].position.y]
                                   [chr->genes[i].position.z];
        if (cell == CELL_UNKNOWN) {
            unknown_cells++;
        }
    }
    fitness += unknown_cells * config->weight_exploration * 0.5;
    
    /* Survivor bonus */
    if (chr->target_type == 1) {
        if (point3d_equals(chr->end_pos, chr->target_pos)) {
            fitness += config->weight_survivor * 5;
        } else {
            double progress = 1.0 - (dist_to_target / (map_euclidean_distance(chr->start_pos, chr->target_pos) + 1));
            fitness += config->weight_survivor * progress;
        }
    }
    
    chr->fitness = fitness;
    return fitness;
}

/*
 * online_ga_tournament
 * Role: select a parent for crossover using tournament selection.
 * How: samples a small subset of the population and returns the
 *       individual with the best fitness among them.
 */
Chromosome* online_ga_tournament(LocalPopulation* pop, const Config* config) {
    (void)config;
    
    if (pop->size == 0) return NULL;
    if (pop->size == 1) return &pop->individuals[0];
    
    Chromosome* best = NULL;
    double best_fitness = -DBL_MAX;
    
    int tournament_size = 3;
    if (tournament_size > pop->size) tournament_size = pop->size;
    
    for (int i = 0; i < tournament_size; i++) {
        int idx = utils_random_int(0, pop->size - 1);
        if (pop->individuals[idx].fitness > best_fitness) {
            best_fitness = pop->individuals[idx].fitness;
            best = &pop->individuals[idx];
        }
    }
    
    return best;
}

/*
 * online_ga_crossover
 * Role: produce child chromosomes from two parents.
 * How: with a chance controlled by `crossover_rate` it splices parent
 *       gene segments at random cut points to form children and then
 *       validates them, reverting to parents if invalid.
 */
void online_ga_crossover(const Chromosome* p1, const Chromosome* p2,
                         Chromosome* c1, Chromosome* c2,
                         Robot* robot, SharedData* shared, const Config* config) {
    /* Default: copy parents */
    memcpy(c1, p1, sizeof(Chromosome));
    memcpy(c2, p2, sizeof(Chromosome));
    
    if (!utils_random_choice(config->crossover_rate)) {
        return;
    }
    
    if (p1->length < 2 || p2->length < 2) {
        return;
    }
    
    int point1 = utils_random_int(1, p1->length - 1);
    int point2 = utils_random_int(1, p2->length - 1);
    
    /* Child 1: splice parents at random cut points.
     * This copies the head of p1 and the tail of p2 into c1. If the
     * generated chromosome is invalid (connectivity/debris), it will be
     * reverted back to the parent copy below.
     */
    int new_len1 = point1 + (p2->length - point2);
    if (new_len1 > 0 && new_len1 < MAX_PATH_LENGTH) {
        for (int i = 0; i < point1 && i < p1->length; i++) {
            c1->genes[i] = p1->genes[i];
        }
        for (int i = point2; i < p2->length; i++) {
            int idx = point1 + (i - point2);
            if (idx < MAX_PATH_LENGTH) {
                c1->genes[idx] = p2->genes[i];
            }
        }
        c1->length = new_len1;
        if (new_len1 > 0) {
            c1->end_pos = c1->genes[new_len1-1].position;
        }
    }
    
    /* Child 2 */
    int new_len2 = point2 + (p1->length - point1);
    if (new_len2 > 0 && new_len2 < MAX_PATH_LENGTH) {
        for (int i = 0; i < point2 && i < p2->length; i++) {
            c2->genes[i] = p2->genes[i];
        }
        for (int i = point1; i < p1->length; i++) {
            int idx = point2 + (i - point1);
            if (idx < MAX_PATH_LENGTH) {
                c2->genes[idx] = p1->genes[i];
            }
        }
        c2->length = new_len2;
        if (new_len2 > 0) {
            c2->end_pos = c2->genes[new_len2-1].position;
        }
    }
    
    /* Validate children immediately to avoid inserting impossible paths. */
    c1->is_valid = online_ga_validate_path(c1, robot, shared);
    c2->is_valid = online_ga_validate_path(c2, robot, shared);

    /* Revert any invalid child back to its corresponding parent to
     * preserve population stability and avoid corrupting the gene pool.
     */
    if (!c1->is_valid) {
        memcpy(c1, p1, sizeof(Chromosome));
    }
    if (!c2->is_valid) {
        memcpy(c2, p2, sizeof(Chromosome));
    }
}

/*
 * online_ga_mutate
 * Role: introduce variation in a chromosome.
 * How: randomly performs a small point mutation (move a gene) or deletes
 *       a step, then re-validates the path against known debris and bounds.
 */
void online_ga_mutate(Chromosome* chr, Robot* robot, SharedData* shared,
                      const Config* config) {
    if (!chr->is_valid || chr->length < 2) return;
    if (!utils_random_choice(config->mutation_rate)) return;
    
    int mutation_type = utils_random_int(0, 1);  /* Simplified mutations */
    
    if (mutation_type == 0 && chr->length > 2) {
        /* Point mutation: replace a middle gene with a nearby neighbor */
        int idx = utils_random_int(1, chr->length - 1);
        Point3D old_pos = chr->genes[idx].position;
        
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            int dir = (d + utils_random_int(0, NUM_DIRECTIONS-1)) % NUM_DIRECTIONS;
            Point3D new_pos = utils_apply_direction(old_pos, dir);
            
            if (new_pos.x >= 0 && new_pos.x < shared->dim_x &&
                new_pos.y >= 0 && new_pos.y < shared->dim_y &&
                new_pos.z >= 0 && new_pos.z < shared->dim_z) {
                
                int cell = robot->known_map[new_pos.x][new_pos.y][new_pos.z];
                if (cell != CELL_DEBRIS) {
                    chr->genes[idx].position = new_pos;
                    chr->genes[idx].direction = dir;
                    break;
                }
            }
        }
    } else if (chr->length > 3) {
        /* Deletion mutation: remove a middle step to shorten/alter topology */
        int idx = utils_random_int(1, chr->length - 2);
        for (int i = idx; i < chr->length - 1; i++) {
            chr->genes[i] = chr->genes[i+1];
        }
        chr->length--;
    }
    
    /* Re-validate after mutation to ensure feasibility */
    chr->is_valid = online_ga_validate_path(chr, robot, shared);
}

/*
 * online_ga_validate_path
 * Role: check that a chromosome's path is feasible with current knowledge.
 * How: ensures all positions are in-bounds, not known debris, and that
 *       consecutive steps are within a connectivity threshold.
 */
int online_ga_validate_path(Chromosome* chr, Robot* robot, SharedData* shared) {
    if (chr->length < 1) return 0;
    
    for (int i = 0; i < chr->length; i++) {
        Point3D pos = chr->genes[i].position;
        
        if (pos.x < 0 || pos.x >= shared->dim_x ||
            pos.y < 0 || pos.y >= shared->dim_y ||
            pos.z < 0 || pos.z >= shared->dim_z) {
            return 0;
        }
        
        /* Only reject if KNOWN to be debris */
        int cell = robot->known_map[pos.x][pos.y][pos.z];
        if (cell == CELL_DEBRIS) {
            return 0;
        }
    }
    
    /* Connectivity check: allow modest gaps (<= 2.5 units) so the GA can
     * consider moves through partially unknown regions, but reject wildly
     * disconnected sequences.
     */
    for (int i = 1; i < chr->length; i++) {
        double dist = map_euclidean_distance(chr->genes[i-1].position,
                                             chr->genes[i].position);
        if (dist > 2.5) {
            return 0;
        }
    }
    
    return 1;
}

/*
 * online_ga_get_best
 * Role: return the best individual in the population.
 * How: returns pointer to the first individual (population kept sorted).
 */
Chromosome* online_ga_get_best(LocalPopulation* pop) {
    if (pop->size == 0) return NULL;
    return &pop->individuals[0];
}

/*
 * online_ga_needs_replan
 * Role: decide whether the robot should run the planner now.
 * How: checks explicit replan flag, path validity/length, next-step blockage,
 *       and a periodic replan interval based on `steps_taken`.
 */
int online_ga_needs_replan(Robot* robot, SharedData* shared, const Config* config) {
    if (robot->needs_replan) {
        robot->needs_replan = 0;  /* Clear flag */
        return 1;
    }
    
    if (!robot->current_path.is_valid || robot->current_path.length == 0) {
        return 1;
    }
    
    if (robot->path_step >= robot->current_path.length) {
        return 1;
    }
    
    /* Check if next step is blocked */
    if (robot->path_step < robot->current_path.length) {
        Point3D next = robot->current_path.genes[robot->path_step].position;
        int cell = robot->known_map[next.x][next.y][next.z];
        if (cell == CELL_DEBRIS) {
            return 1;
        }
    }
    
    /* Periodic replan */
    if (config->replan_interval > 0 && 
        robot->steps_taken > 0 && 
        robot->steps_taken % config->replan_interval == 0) {
        return 1;
    }
    
    return 0;
}

/*
 * online_ga_plan
 * Role: top-level planner invoked by a robot to produce `current_path`.
 * How: initializes a population, evolves it for up to `local_max_generations`,
 *       and installs the best valid chromosome into `robot->current_path`.
 */
void online_ga_plan(Robot* robot, SharedData* shared, const Config* config) {
    LocalPopulation pop;
    
    online_ga_init(&pop, robot, shared, config);
    
    /* Run evolution */
    for (int gen = 0; gen < config->local_max_generations; gen++) {
        online_ga_iterate(&pop, robot, shared, config);
        
        /* Early termination */
        if (pop.best_fitness > 20 && gen > 3) {
            break;
        }
    }
    
    Chromosome* best = online_ga_get_best(&pop);
    if (best && best->is_valid && best->length > 0) {
        memcpy(&robot->current_path, best, sizeof(Chromosome));
        robot->path_step = 0;
        shared->stats.total_ga_runs++;
    }
}

