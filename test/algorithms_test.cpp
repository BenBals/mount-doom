#include "algorithms_test_helpers.h"
#include "fmt/core.h"
#include "src/algorithms.h"
#include "src/datastructures.h"
#include "src/executable_helpers.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <numeric>
#include <random>

using namespace algorithms;
using namespace datastructures;

// seeded randomness
static std::mt19937 gen(42);
static auto test_root = executable_helpers::find_vcs_root() / "instances/test";

bool is_list_of_original_indices_feasible_dfvs(const FvsInstance &instance,
                                               const std::vector<size_t> &solution_vec) {
  auto graph_copy = instance._graph;
  for (size_t idx = 0; idx < instance._graph.size(); idx++) {
    if (std::find(solution_vec.begin(), solution_vec.end(), instance._original_indices[idx]) !=
        solution_vec.end()) {
      graph_copy.isolate_node(idx);
    }
  }
  return graph_copy.is_acyclic();
}

bool is_partial_solution_feasible(const FvsInstance &instance, PartialSolution solution) {
  return is_list_of_original_indices_feasible_dfvs(instance, solution.solution_nodes());
}

void solver_gives_feasible_solution_and_optimal_if_known(FvsInstance instance, Solver *solver,
                                                         const std::string &solver_name) {
  spdlog::info("start solving {}", instance._name);
  auto initial_instance = instance;
  auto solution = solver->solve(std::move(instance));

  ASSERT_TRUE(solution.has_value())
      << instance._name << " could not be solved by " << solver_name << "!";

  ASSERT_TRUE(is_partial_solution_feasible(initial_instance, solution.value()))
      << instance._name << " is not solved (i.e. solution is no DFVS)!";
  if (initial_instance.known_solution_size().has_value()) {
    ASSERT_EQ(solution.value().solution_nodes().size(),
              initial_instance.known_solution_size().value())
        << instance._name << " has a known solution of size "
        << initial_instance.known_solution_size().value() << " but a solution of size "
        << solution.value().solution_nodes().size() << " was found!";
  }
}

void branch_and_bound_gives_feasible_solution_and_optimal_if_known(FvsInstance instance) {
  auto solver = std::make_unique<algorithms::BranchingChallengeSolver>();

  return solver_gives_feasible_solution_and_optimal_if_known(std::move(instance), solver.get(),
                                                             "BranchingChallenge");
}

void brute_force_gives_feasible_solution_and_optimal_if_known(FvsInstance instance) {
  auto solver = std::make_unique<algorithms::BruteforcingSolver>();
  return solver_gives_feasible_solution_and_optimal_if_known(std::move(instance), solver.get(),
                                                             "BruteForce");
}

void solver_agree_on_solution_size(FvsInstance &&instance, Solver *solver1,
                                   const std::string &name1, Solver *solver2,
                                   const std::string &name2) {
  auto instance_name = instance._name;
  auto copy = instance;
  auto solution1 = solver1->solve(std::move(copy));
  ASSERT_TRUE(solution1.has_value()) << name1 << " could not solve instance " << instance_name;

  auto solution2 = solver2->solve(std::move(instance));
  ASSERT_TRUE(solution2.has_value()) << name2 << " could not solve instance " << instance_name;

  ASSERT_EQ(solution1->number_of_final_nodes(), solution2->number_of_final_nodes())
      << "Solvers " << name1 << " and " << name2 << " have different solution sizes on "
      << instance_name;
}

TEST(BruteForce, TestGraphsSolutionsAreFeasibleAndOptimalIfKnown) {
  for (const auto &entry : std::filesystem::directory_iterator(test_root)) {
    brute_force_gives_feasible_solution_and_optimal_if_known(read_instance_from_file(entry.path()));
  }
}

TEST(Branching, TestGraphsSolutionsAreFeasibleAndOptimalIfKnown) {
  for (const auto &entry : std::filesystem::directory_iterator(test_root)) {
    branch_and_bound_gives_feasible_solution_and_optimal_if_known(
        read_instance_from_file(entry.path()));
  }
}

TEST(Branching, ExampleExactGraphsSolutionsAreFeasible) {
  auto exact_root = executable_helpers::find_vcs_root() / "instances/exact_public";
  std::vector<std::string> files = {"e_001", "e_025", "e_091"};
  for (const auto &file : files) {
    branch_and_bound_gives_feasible_solution_and_optimal_if_known(
        read_instance_from_file(exact_root / file));
  }
}

TEST(Branching, RandomOverlappingCyclesAreOptimal) {
  // This does not require branching. Sad.
  for (size_t i = 1; i <= 10; i++) {
    size_t num_cycles = i * 50, cycle_size = std::max(static_cast<size_t>(gen()) % 50, 3ul),
           random_arcs = i * 1000;
    FvsInstance instance = generate_overlapping_cycles(num_cycles, cycle_size, random_arcs);
    branch_and_bound_gives_feasible_solution_and_optimal_if_known(instance);
  }
}
TEST(Branching, DISABLED_WideCyclesAreOptimal) {
  for (size_t i = 1; i <= 10; i++) {
    size_t perimeter = i * 5, width = i * 3;
    FvsInstance instance = generate_wide_cycle(perimeter, width);
    branch_and_bound_gives_feasible_solution_and_optimal_if_known(instance);
  }
}

void vertex_cover_like_are_optimal_impl(size_t num_nodes_multiplicator) {
  for (size_t i = 1; i <= 7; i++) {
    size_t num_nodes = i * num_nodes_multiplicator;
    size_t extra_arcs = num_nodes * num_nodes / 8;

    FvsInstance instance = generate_vertex_cover_like(num_nodes, extra_arcs, i);
    branch_and_bound_gives_feasible_solution_and_optimal_if_known(instance);
  }
}

TEST(Branching, VertexCoverLikeAreOptimal_Small) { vertex_cover_like_are_optimal_impl(10); }

TEST(Branching, ArticulationPointAndBranchingAgreeOnSmallGraphs) {
  gen.seed(1);
  auto branching_solver = std::make_unique<BranchingChallengeSolver>();

  auto articulation_point_solver = std::make_unique<ArticulationPointSolver>();
  articulation_point_solver->applicable_solver(articulation_point_solver.get());
  articulation_point_solver->not_applicable_solver(branching_solver.get());

  for (size_t i = 1; i <= 500; i++) {
    auto num_nodes = std::max(gen() % 20ul, 2ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    spdlog::info("Checking for cut node (branching challenge) with n={}, m={}, seed={}", num_nodes,
                 num_arcs, i);

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    auto num_cut_nodes = 1 + (gen() % 3); // don't cut too many nodes
    // this might not create a cut node, but increases probability
    for (size_t cut_node_cnt = 0; cut_node_cnt < num_cut_nodes; cut_node_cnt++) {
      size_t cut_position = gen() % num_nodes;
      for (size_t left = 0; left < cut_position; left++) {
        for (size_t right = cut_position + 1; right < num_nodes; right++) {
          if (instance._graph.is_arc(left, right)) {
            instance._graph.remove_arc_unsafe(left, right);
          }
          if (instance._graph.is_arc(right, left)) {
            instance._graph.remove_arc_unsafe(right, left);
          }
        }
      }
    }
    instance.permute();

    solver_agree_on_solution_size(std::move(instance), branching_solver.get(), "BranchingChallenge",
                                  articulation_point_solver.get(),
                                  "ArticulationPoint(BranchingChallenge)");
  }
}
TEST(Branching, ArticulationPointChallengeAndBranchingChallengeAgreeOnSmallGraphs) {
  gen.seed(1);
  auto branching_solver = std::make_unique<BranchingChallengeSolver>();

  auto articulation_point_solver = std::make_unique<ArticulationPointBranchingChallengeSolver>();

  for (size_t i = 1; i <= 500; i++) {
    auto num_nodes = std::max(gen() % 20ul, 2ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    spdlog::info("Checking for articulation point challenge solver with n={}, m={}, seed={}",
                 num_nodes, num_arcs, i);

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    solver_agree_on_solution_size(std::move(instance), branching_solver.get(), "BranchingChallenge",
                                  articulation_point_solver.get(),
                                  "ArticulationPointBranchingChallenge");
  }
}
TEST(Branching, VertexCoverChallengeAndArticulationPointChallengeAgreeOnSmallGraphs) {
  gen.seed(1);
  auto articulation_point_solver = std::make_unique<ArticulationPointBranchingChallengeSolver>();
  auto vertex_cover_solver =
      std::make_unique<ArticulationPointVertexCoverBranchingChallengeSolver>();

  for (size_t i = 1; i <= 10000; i++) {
    auto num_nodes = std::max(gen() % 20ul, 2ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    spdlog::info("Checking for articulation point vertex cover challenge with n={}, m={}, seed={}",
                 num_nodes, num_arcs, i);

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    solver_agree_on_solution_size(std::move(instance), articulation_point_solver.get(),
                                  "ArticulationPointBranchingChallenge", vertex_cover_solver.get(),
                                  "ArticulationPointVertexCoverBranchingChallenge");
  }
}

/**
 * During testing it occurred, that taking the nodes into the overall partial solution was not done
 * correctly.
 */
TEST(Branching, CutVertexSolverRegressionTakeCorrectNodes) {
  auto instance =
      datastructures::read_instance_from_file(test_root / "t_articulation_point_solver_regression");

  auto branching_solver = std::make_unique<BranchingChallengeSolver>();
  auto cut_vertex_solver = std::make_unique<ArticulationPointSolver>();
  cut_vertex_solver->applicable_solver(cut_vertex_solver.get());
  cut_vertex_solver->not_applicable_solver(branching_solver.get());

  solver_agree_on_solution_size(std::move(instance), branching_solver.get(), "BranchingChallenge",
                                cut_vertex_solver.get(), "CutNode(BranchingChallenge)");
}
TEST(LowerBound, LowerBoundDoesNotExceedSolutionSizeOnTestInstances) {
  for (const auto &entry : std::filesystem::directory_iterator(test_root)) {
    auto instance = read_instance_from_file(entry.path());
    instance.collect_self_loops_into_partial_fvs();
    const auto instance_copy = instance;

    BranchingChallengeSolver solver;
    auto solution = solver.solve(std::move(instance));

    ASSERT_TRUE(solution.has_value()) << "BranchingChallengeSolver did not find a value";
    ASSERT_LE(instance_copy.partial_solution_size() + instance_copy.lower_bound(),
              solution->number_of_final_nodes())
        << "lower bound must not exceed solution size, but did on " << instance_copy._name;
  }
}

TEST(LowerBound, LowerBoundZeroIffSolvedOnTestInstances) {
  for (const auto &entry : std::filesystem::directory_iterator(test_root)) {
    auto instance = read_instance_from_file(entry.path());
    instance.collect_self_loops_into_partial_fvs();
    if (is_partial_solution_feasible(instance, instance._partial_solution)) {
      ASSERT_EQ(instance.lower_bound(), 0)
          << "Instance " << instance._name << " is solved, but found a non-zero lower bound";
    } else {
      ASSERT_GT(instance.lower_bound(), 0)
          << "Instance " << instance._name << " has loop, but a lower bound of 0 is found";
    }
  }
}

TEST(LowerBound, LowerBoundDoesNotExceedSolutionSizeOnSmallRandomGraphs) {
  gen.seed(1);
  for (size_t i = 1; i <= 10000; i++) {
    auto num_nodes = std::max(gen() % 20ul, 2ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    instance.collect_self_loops_into_partial_fvs();
    const auto instance_copy = instance;

    BranchingChallengeSolver solver;
    auto solution = solver.solve(std::move(instance));

    ASSERT_TRUE(solution.has_value()) << "BranchingChallengeSolver did not find a value";
    ASSERT_LE(instance_copy.partial_solution_size() + instance_copy.lower_bound(),
              solution->number_of_final_nodes())
        << "lower bound must not exceed solution size, but did on " << instance_copy._name;
  }
}

TEST(LowerBound, LowerBoundZeroIffSolvedOnSmallRandomGraphs) {
  gen.seed(1);
  for (size_t i = 1; i <= 10000; i++) {
    auto num_nodes = std::max(gen() % 20ul, 2ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    instance.collect_self_loops_into_partial_fvs();
    if (is_partial_solution_feasible(instance, instance._partial_solution)) {
      ASSERT_EQ(instance.lower_bound(), 0)
          << "Instance " << instance._name << " is solved, but found a non-zero lower bound";
    } else {
      ASSERT_GT(instance.lower_bound(), 0)
          << "Instance " << instance._name << " has loop, but a lower bound of 0 is found";
    }
  }
}

TEST(Branching, CycleCliqueVertexCoverAndBranchingChallengeAgreeOnSmallGraphs) {
  gen.seed(1);
  auto branching_solver = std::make_unique<BranchingChallengeSolver>();

  auto clique_cycle_vc_solver = std::make_unique<CycleCliqueVertexCoverSolver>();

  for (size_t i = 1; i <= 5000; i++) {
    auto num_nodes = std::max(gen() % 20ul, 2ul);
    auto num_arcs = gen() % num_nodes * num_nodes;

    spdlog::info("Checking for cycle clique solver with n={}, m={}, seed={}", num_nodes, num_arcs,
                 i);

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    solver_agree_on_solution_size(std::move(instance), branching_solver.get(), "BranchingChallenge",
                                  clique_cycle_vc_solver.get(), "CycleCliqueVertexCover");
  }
}

TEST(Branching, GreedyHeuristicSolverGivesFeasibleSolution) {
  gen.seed(1);
  for (size_t i = 1; i <= 10000; i++) {
    auto num_nodes = std::max(gen() % 20ul, 2ul);
    auto num_arcs = gen() % (num_nodes * (num_nodes - 1) / 2);
    spdlog::info("Checking greedy heuristic solver is feasible with n={}, m={}, seed={}", num_nodes,
                 num_arcs, i);
    auto instance = gen_random_instance(num_nodes, num_arcs, i, false);

    auto instance_backup = instance;

    auto instance_copy = instance;
    auto instance_copy2 = instance;

    MaxByDegreeHeuristicSolver solver;
    auto unfiltered_solution = solver.greedily_find_initial_solution(std::move(instance));
    auto filtered_solution_minimal = solver.make_solution_nodes_minimal(
        std::move(instance_copy), unfiltered_solution.solution_nodes());

    ASSERT_TRUE(is_partial_solution_feasible(instance_backup, filtered_solution_minimal))
        << " greedy heuristic minimal solver did not produce a feasible solution";

    auto filtered_solution = solver.iterative_small_compression(
        std::move(instance_copy2), filtered_solution_minimal.solution_nodes());

    ASSERT_TRUE(is_partial_solution_feasible(instance_backup, filtered_solution))
        << " greedy heuristic solver did not produce a feasible solution";
  }
}

TEST(Branching, GreedyHeuristicSolverWithReductionsGivesFeasibleSolution) {
  gen.seed(1);
  for (size_t i = 1; i <= 10000; i++) {
    auto num_nodes = std::max(gen() % 20ul, 3ul);
    auto num_arcs = gen() % num_nodes * num_nodes;
    auto reduce_after = 1 + gen() % (num_nodes / 3);

    MaxByDegreeHeuristicSolver solver(reduce_after);
    spdlog::info("Checking greedy heuristic solver reducing after taking {} is feasible with n={}, "
                 "m={}, seed={}",
                 reduce_after, num_nodes, num_arcs, i);

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    auto instance_copy = instance;

    auto solution = solver.solve(std::move(instance));
    ASSERT_TRUE(solution.has_value()) << " greedy heuristic solver did not produce a solution";

    ASSERT_TRUE(is_partial_solution_feasible(instance_copy, solution.value()))
        << " greedy heuristic solver did not produce a feasible solution";
  }
}

// with introduction of degree 2 folding, the solution to this graph was not feasible anymore
TEST(Greedy, GreedyGivesFeasibleSolutionWithReductionsRegression) {
  FvsInstance instance =
      read_instance_from_file(test_root / "t_greedy_feasible_with_reductions_regression");
  auto instance_copy = instance;

  MaxByDegreeHeuristicSolver solver(1);
  auto solution = solver.solve(std::move(instance));
  ASSERT_TRUE(solution.has_value()) << " greedy heuristic solver did not produce a solution";

  ASSERT_TRUE(is_partial_solution_feasible(instance_copy, solution.value()))
      << " greedy heuristic solver did not produce a feasible solution";
}

TEST(Branching, GreedyHeuristicSmallIterativeCompressionGives_2_1_MinimalSolution) {
  for (size_t i = 1; i <= 100; i++) {
    auto num_nodes = std::max(gen() % 40ul, 5ul);
    auto num_arcs = gen() % num_nodes * num_nodes;
    auto reduce_after = 1 + gen() % (num_nodes / 3);

    MaxByDegreeHeuristicSolver solver(reduce_after);
    spdlog::info("Checking greedy heuristic solver reducing after taking {} is gives a "
                 "2-1-compression minimal solution with n={}, "
                 "m={}, seed={}",
                 reduce_after, num_nodes, num_arcs, i);

    auto instance = generate_random_instance(num_nodes, num_arcs, i);
    auto instance_copy = instance;
    auto solution = solver.solve(std::move(instance));
    auto real_solution_nodes = solution->solution_nodes();

    for (size_t first_node_idx = 0; first_node_idx < real_solution_nodes.size(); first_node_idx++) {
      for (size_t second_node_idx = 0; second_node_idx < real_solution_nodes.size();
           second_node_idx++) {
        if (first_node_idx == second_node_idx)
          continue;
        for (size_t new_node_idx = 0; new_node_idx < instance_copy._graph.size(); new_node_idx++) {
          auto new_node = instance_copy._original_indices[new_node_idx];
          auto real_solution_copy = real_solution_nodes;
          real_solution_copy[first_node_idx] = new_node;
          std::swap(real_solution_copy[second_node_idx], real_solution_copy.back());
          real_solution_copy.pop_back();

          ASSERT_FALSE(is_list_of_original_indices_feasible_dfvs(instance_copy, real_solution_copy))
              << "No 2-1-compression minimal solution returned by greedy solver";
        }
      }
    }
  }
}
