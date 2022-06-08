#include "algorithms_test_helpers.h"
#include "src/algorithms.h"
#include "src/datastructures.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <executable_helpers.h>
#include <numeric>
#include <random>

using namespace algorithms;
using namespace datastructures;

static std::mt19937 gen(42);
static auto test_root = executable_helpers::find_vcs_root() / "instances/test";

TEST(Branching, VertexCoverLikeAreOptimal_Large) { vertex_cover_like_are_optimal_impl(30); }

TEST(Branching, BruteForceAndBranchingAgreeOnSmallGraphs) {
  auto branching_solver = std::make_unique<BranchingChallengeSolver>();
  auto bruteforce_solver = std::make_unique<BruteforcingSolver>();

  for (size_t i = 1; i <= 50; i++) {
    auto num_nodes = std::max(gen() % 15ul, 2ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    spdlog::info("Brute force check for branching challenge with n={}, m={}, seed={}", num_nodes,
                 num_arcs, i);

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    solver_agree_on_solution_size(std::move(instance), branching_solver.get(), "BranchingChallenge",
                                  bruteforce_solver.get(), "Bruteforce");
  }
}

void check_reduction_rules_keep_solution_size_and_feasibility(FvsInstance &&instance) {
  const auto instance_copy = instance;

  auto bruteforce_solver = std::make_unique<BruteforcingSolver>();
  auto brute_force_solution = bruteforce_solver->solve(std::move(instance));
  auto check_rule = [&brute_force_solution, &bruteforce_solver,
                     &instance_copy](FvsInstance &&reduced) {
    auto brute_force_after_reduced = bruteforce_solver->solve(std::move(reduced));

    ASSERT_TRUE(brute_force_solution.has_value()) << "bruteforce did not return a solution";
    ASSERT_TRUE(brute_force_after_reduced.has_value())
        << "bruteforce after reductions did not return a solution";

    // check that applying finalizers keeps solution size
    ASSERT_EQ(brute_force_solution->number_of_final_nodes(),
              brute_force_after_reduced->number_of_final_nodes())
        << "number_of_final_nodes() differs before finalizers";
    ASSERT_EQ(brute_force_solution->solution_nodes().size(),
              brute_force_after_reduced->solution_nodes().size())
        << "solution_nodes().size() differs";
    ASSERT_EQ(brute_force_solution->number_of_final_nodes(),
              brute_force_after_reduced->number_of_final_nodes())
        << "number_of_final_nodes() differs after finalizers";

    ASSERT_TRUE(is_partial_solution_feasible(instance_copy, brute_force_solution.value()))
        << "bruteforce solution was not feasible";
    ASSERT_TRUE(is_partial_solution_feasible(instance_copy, brute_force_after_reduced.value()))
        << "bruteforce solution was not feasible after reductions";
  };

  auto reduced_instance = instance_copy;

  reduced_instance.reduction_remove_isolated_nodes();
  check_rule(std::move(reduced_instance));

  reduced_instance = instance_copy;
  reduced_instance.reduction_shortcut_clique_adjacent_nodes();
  check_rule(std::move(reduced_instance));

  reduced_instance = instance_copy;
  reduced_instance.reduction_fold_degree_2_nodes();
  check_rule(std::move(reduced_instance));

  reduced_instance = instance_copy;
  reduced_instance.reduction_remove_dominated_edges();
  check_rule(std::move(reduced_instance));

  reduced_instance = instance_copy;
  reduced_instance.reduction_delete_arcs_between_strongly_connected_components_ignoring_k2();
  check_rule(std::move(reduced_instance));

  reduced_instance = instance_copy;
  reduced_instance.reduction_funnel();
  check_rule(std::move(reduced_instance));

  reduced_instance = instance_copy;
  reduced_instance.reduction_funnel(SIZE_MAX, false); // without folding
  check_rule(std::move(reduced_instance));

  reduced_instance = instance_copy;
  reduced_instance.collect_self_loops_into_partial_fvs();
  check_rule(std::move(reduced_instance));

  reduced_instance = instance_copy;
  reduced_instance.reduction_shortcut_directed_in_and_out_degree_1();
  check_rule(std::move(reduced_instance));

  reduced_instance = instance_copy;
  reduced_instance.reduction_pick_undirected_neighbor_if_undirected_neighborhood_is_superset();
  check_rule(std::move(reduced_instance));
}

TEST(BruteForce, ReductionRulesKeepSolutionSizeAndFeasibility) {
  for (size_t i = 1; i <= 50; i++) {
    auto num_nodes = std::max(gen() % 15ul, 2ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    spdlog::info("Brute force check for reduction rules with n={}, m={}, seed={}", num_nodes,
                 num_arcs, i);

    check_reduction_rules_keep_solution_size_and_feasibility(
        generate_random_instance(num_nodes, num_arcs, i));
  }
}

TEST(BruteForec, FunnelFoldTest) {
  auto instance = read_instance_from_file(test_root / "t_pre_funnel_reduction");
  check_reduction_rules_keep_solution_size_and_feasibility(std::move(instance));
}

TEST(BruteForce, ReductionRulesKeepSolutionSizeAndFeasibilityTestInstances) {
  for (const auto &entry : std::filesystem::directory_iterator(test_root)) {
    spdlog::info("Brute force check for reduction rules with {}", entry.path().filename().string());
    check_reduction_rules_keep_solution_size_and_feasibility(read_instance_from_file(entry.path()));
  }
}

TEST(BruteForce, DegreeTwoReductionRulesKeepSolutionSizeAndFeasibility) {
  auto add_deg_two_foldable = [](FvsInstance &instance) {
    // create new nodes at end
    auto initial_num_nodes = instance._graph.size();
    auto middle = instance.create_node(initial_num_nodes + 1, false);
    auto neighbor0 = instance.create_node(initial_num_nodes + 2, false);
    auto neighbor1 = gen() % initial_num_nodes;

    instance._graph.add_arc(middle, neighbor0);
    instance._graph.add_arc(middle, neighbor1);
    instance._graph.add_arc(neighbor0, middle);
    instance._graph.add_arc(neighbor1, middle);

    for (size_t idx = 0; idx < initial_num_nodes; idx++) {
      if (gen() % (initial_num_nodes / 3) == 0) {
        instance._graph.add_arc(idx, neighbor0);
        instance._graph.add_arc(neighbor0, idx);
      }
    }
  };
  for (size_t i = 1; i <= 50; i++) {

    gen.seed(i);
    auto num_nodes = std::max(gen() % 11ul, 3ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    spdlog::info("Brute force check for degree two reduction rules with n={}, m={}, seed={}",
                 num_nodes, num_arcs, i);

    auto instance = generate_random_instance(num_nodes, num_arcs, i);

    add_deg_two_foldable(instance);
    add_deg_two_foldable(instance);
    add_deg_two_foldable(instance);
    instance.permute();

    check_reduction_rules_keep_solution_size_and_feasibility(std::move(instance));
  }
}
