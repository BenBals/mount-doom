#pragma once

#include "src/algorithms.h"
#include "src/datastructures.h"
#include <string>

bool is_list_of_original_indices_feasible_dfvs(const FvsInstance &instance,
                                               const std::vector<size_t> &solution_vec);

bool is_partial_solution_feasible(const FvsInstance &instance, PartialSolution solution);

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
