#include "algorithms_test_helpers.h"
#include "src/algorithms.h"
#include "src/datastructures.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <numeric>
#include <random>

using namespace algorithms;
using namespace datastructures;

static std::mt19937 gen(42);

TEST(Branching, VertexCoverLikeAreOptimal_Large) { vertex_cover_like_are_optimal_impl(30); }

TEST(Branching, BruteForceAndBranchingAgreeOnSmallGraphs) {
  auto branching_solver = std::make_unique<BranchingChallengeSolver>();
  auto bruteforce_solver = std::make_unique<BruteforcingSolver>();

  for (size_t i = 1; i <= 20; i++) {
    auto num_nodes = std::max(gen() % 20ul, 2ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    spdlog::info("Brute force check for branching challenge with n={}, m={}, seed={}", num_nodes,
                 num_arcs, i);

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    solver_agree_on_solution_size(std::move(instance), branching_solver.get(), "BranchingChallenge",
                                  bruteforce_solver.get(), "Bruteforce");
  }
}

TEST(BruteForce, ReductionRulesKeepSolutionSize) {
  for (size_t i = 1; i <= 20; i++) {

    auto num_nodes = std::max(gen() % 20ul, 2ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    spdlog::info("Brute force check for reduction rules with n={}, m={}, seed={}", num_nodes,
                 num_arcs, i);

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    auto instance_copy = instance;

    auto bruteforce_solver = std::make_unique<BruteforcingSolver>();
    auto brute_force_solution = bruteforce_solver->solve(std::move(instance));
    auto check_rule = [&brute_force_solution, &bruteforce_solver](FvsInstance &&reduced) {
      auto brute_force_after_reduced = bruteforce_solver->solve(std::move(reduced));

      ASSERT_TRUE(brute_force_solution.has_value());
      ASSERT_EQ(brute_force_solution.value().size(), brute_force_after_reduced.value().size());
    };

    auto reduced_instance = instance_copy;

    reduced_instance.reduction_remove_isolated_nodes();
    check_rule(std::move(reduced_instance));

    reduced_instance = instance_copy;
    reduced_instance.reduction_delete_arcs_between_strongly_connected_components_ignoring_k2();
    check_rule(std::move(reduced_instance));

    reduced_instance = instance_copy;
    reduced_instance.collect_self_loops_into_partial_fvs();
    check_rule(std::move(reduced_instance));
  }
}
