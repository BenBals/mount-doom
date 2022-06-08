#pragma once

#include "src/algorithms.h"
#include "src/datastructures.h"
#include <string>

void solver_gives_feasible_solution_and_optimal_if_known(datastructures::FvsInstance instance,
                                                         algorithms::Solver *solver,
                                                         const std::string &solver_name);

void branch_and_bound_gives_feasible_solution_and_optimal_if_known(
    datastructures::FvsInstance instance);

void brute_force_gives_feasible_solution_and_optimal_if_known(datastructures::FvsInstance instance);

void solver_agree_on_solution_size(datastructures::FvsInstance &&instance,
                                   algorithms::Solver *solver1, const std::string &name1,
                                   algorithms::Solver *solver2, const std::string &name2);

void vertex_cover_like_are_optimal_impl(size_t num_nodes_multiplicator);

