#include "src/algorithms/fvs_generation.h"
#include "src/datastructures.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "src/executable_helpers.h"
#include <random>

// seeded randomness
std::mt19937 gen(42);

using namespace datastructures;

auto test_root = executable_helpers::find_vcs_root() / "instances/test";

TEST(GraphImport, TestGraph1) {
  Graph graph = read_instance_from_file(test_root / "t_001")._graph;

  ASSERT_EQ(graph.size(), 5);

  ASSERT_TRUE(graph[0].out_edges().empty());
  ASSERT_EQ(graph[0].in_edges().size(), 2);

  ASSERT_TRUE(graph.is_arc(1, 0));
  ASSERT_TRUE(graph.is_arc(1, 2));
  ASSERT_TRUE(graph.is_arc(1, 3));
  ASSERT_EQ(graph[1].out_edges().size(), 3);
  ASSERT_EQ(graph[1].in_edges().size(), 1);

  ASSERT_TRUE(graph[2].out_edges().empty());
  ASSERT_EQ(graph[2].in_edges().size(), 1);

  ASSERT_TRUE(graph.is_arc(3, 1));
  ASSERT_TRUE(graph.is_arc(3, 4));
  ASSERT_EQ(graph[3].out_edges().size(), 2);
  ASSERT_EQ(graph[3].in_edges().size(), 2);

  ASSERT_TRUE(graph.is_arc(4, 0));
  ASSERT_TRUE(graph.is_arc(4, 3));
  ASSERT_EQ(graph[4].out_edges().size(), 2);
  ASSERT_EQ(graph[4].in_edges().size(), 1);
}

TEST(FvsInstanceStats, TestGraph1) {
  FvsInstance instance = read_instance_from_file(test_root / "t_001");

  FvsInstanceStats stats(instance);

  ASSERT_EQ(stats.num_nodes, 5);
  ASSERT_EQ(stats.num_arcs, 7);
  ASSERT_EQ(stats.out_degree_quartiles[0], 0);
  ASSERT_EQ(stats.out_degree_quartiles[1], 0);
  ASSERT_EQ(stats.out_degree_quartiles[2], 2);
  ASSERT_EQ(stats.out_degree_quartiles[3], 2);
  ASSERT_EQ(stats.out_degree_quartiles[4], 3);
}

TEST(GraphOperations, TransposeExample2) {
  Graph before = read_instance_from_file(test_root / "t_002")._graph;
  Graph after = read_instance_from_file(test_root / "t_002_transposed")._graph;

  before.transpose();

  ASSERT_EQ(before, after);
}

TEST(GraphOperations, TransposeByArcSwap) {
  Graph before = read_instance_from_file(test_root / "t_002")._graph;
  Graph after = read_instance_from_file(test_root / "t_002_transposed")._graph;

  before.swap_arc_unsafe(1, 0);
  before.swap_arc_unsafe(1, 2);
  before.swap_arc_unsafe(1, 3);
  before.swap_arc_unsafe(3, 1);
  before.swap_arc_unsafe(3, 4);
  before.swap_arc_unsafe(4, 0);
  before.swap_arc_unsafe(4, 3);
  before.swap_arc_unsafe(5, 0);
  before.swap_arc_unsafe(5, 1);

  ASSERT_EQ(before, after);
}

TEST(GraphReductions, RemoveIsolatedKeepsSelfLoops) {
  FvsInstance before = read_instance_from_file(test_root / "t_004_selfloop");
  Graph after = read_instance_from_file(test_root / "t_004_selfloop")._graph;

  before.reduction_remove_isolated_nodes();

  ASSERT_EQ(before._graph, after);
}

TEST(GraphReductions, RemoveNodesWithInOrOutDegreeOneExample) {
  FvsInstance before = read_instance_from_file(test_root / "t_003");
  Graph after = read_instance_from_file(test_root / "t_003_without_nodes_with_degree_one")._graph;

  before.reduction_shortcut_clique_adjacent_nodes(1);
  before.reduction_remove_isolated_nodes();

  ASSERT_EQ(before._graph, after);
}

TEST(GraphReductions, RemoveEdgesBetweenSCCsExample5) {
  FvsInstance before = read_instance_from_file(test_root / "t_005");
  Graph after = read_instance_from_file(test_root / "t_005_no_arcs_between_sccs")._graph;

  before.reduction_delete_arcs_between_strongly_connected_components_ignoring_k2();

  ASSERT_EQ(before._graph, after);
}

TEST(GraphReductions, RemoveEdgesBetweenSCCsExampleWithk2) {
  FvsInstance before = read_instance_from_file(test_root / "t_pre_remove_k2_sccs");
  Graph after = read_instance_from_file(test_root / "t_post_remove_k2_sccs")._graph;

  before.reduction_delete_arcs_between_strongly_connected_components_ignoring_k2();

  ASSERT_EQ(before._graph, after);
}

TEST(GraphReductions, ShortcutAdjacentToClique) {
  FvsInstance before = read_instance_from_file(test_root / "t_pre_clique_reduction");
  Graph after = read_instance_from_file(test_root / "t_post_clique_reduction")._graph;

  before.reduction_shortcut_clique_adjacent_nodes();

  ASSERT_EQ(before._graph, after);
}

TEST(GraphReductions, DegreeTwoFolding) {
  FvsInstance before = read_instance_from_file(test_root / "t_pre_fold_degree_2_001");
  Graph after = read_instance_from_file(test_root / "t_post_fold_degree_2_001")._graph;

  before.reduction_fold_degree_2_nodes();

  ASSERT_EQ(before._graph, after);
}

TEST(GraphReductions, DegreeTwoFoldingCorrectSolution) {
  FvsInstance before = read_instance_from_file(test_root / "t_pre_fold_degree_2_001");

  before.reduction_fold_degree_2_nodes();

  ASSERT_EQ(before._partial_solution.number_of_final_nodes(), 1);
  ASSERT_EQ(before._partial_solution.solution_nodes(), std::vector<size_t>{2});
  ASSERT_EQ(before._partial_solution.number_of_final_nodes(), 1);
}

TEST(GraphReductions, RemoveDominatedEdge) {
  FvsInstance before = read_instance_from_file(test_root / "t_pre_dominated_edge_reduction");
  Graph after = read_instance_from_file(test_root / "t_post_dominated_edge_reduction")._graph;

  before.reduction_remove_dominated_edges();

  ASSERT_EQ(before._graph, after);
}

TEST(GraphReductions, PickNodeIfSuperset) {
  FvsInstance before = read_instance_from_file(test_root / "t_pre_dominating_node");
  Graph after = read_instance_from_file(test_root / "t_post_dominating_node")._graph;

  before.reduction_pick_undirected_neighbor_if_undirected_neighborhood_is_superset();

  ASSERT_EQ(before._graph, after);
}

TEST(GraphReductions, PickNodeIfSupersetSelfLoop) {
  FvsInstance before = read_instance_from_file(test_root / "t_dominating_node_self_loop");

  before.reduction_pick_undirected_neighbor_if_undirected_neighborhood_is_superset();

  std::vector<size_t> solution = {1};
  ASSERT_EQ(before._partial_solution.solution_nodes(), solution);
}

TEST(GraphReductions, ShortcutDegreeOneDirected) {
  FvsInstance before = read_instance_from_file(test_root / "t_pre_shortcut_directed_degree_one");
  Graph after = read_instance_from_file(test_root / "t_post_shortcut_directed_degree_one")._graph;

  before.reduction_shortcut_directed_in_and_out_degree_1();

  ASSERT_EQ(before._graph, after);
}

TEST(GraphReductions, FunnelReductionNoFolding) {
  FvsInstance before = read_instance_from_file(test_root / "t_pre_funnel_reduction");
  Graph after = read_instance_from_file(test_root / "t_post_funnel_reduction_no_folding")._graph;

  before.reduction_funnel(SIZE_MAX, false);

  ASSERT_EQ(before._graph, after);
}

TEST(GraphReductions, SplittingSCCs) {
  FvsInstance before = read_instance_from_file(test_root / "t_005");
  auto [split, _solution] = split_fvs_instance_by_sccs(std::move(before));

  Graph after1 = read_instance_from_file(test_root / "t_005_scc_1")._graph,
        after2 = read_instance_from_file(test_root / "t_005_scc_2")._graph,
        after3 = read_instance_from_file(test_root / "t_005_scc_3")._graph;
  std::vector<Graph> after = {after1, after2, after3}, split_graphs;

  for (auto inst : split)
    split_graphs.emplace_back(std::move(inst._graph));

  ASSERT_TRUE(std::is_permutation(split_graphs.begin(), split_graphs.end(), after.begin()));
}

TEST(GraphReductions, DeleteNode) {
  FvsInstance before = read_instance_from_file(test_root / "t_006");
  Graph after = read_instance_from_file(test_root / "t_006_node_3_removed")._graph;

  before.remove_nodes([](auto &node) { return node.index() == 2; });

  ASSERT_EQ(before._graph, after);
}
TEST(GraphOperation, BuildExpandedGraph) {
  Graph before = read_instance_from_file(test_root / "t_009")._graph,
        after_expanded = read_instance_from_file(test_root / "t_009_expanded_graph")._graph;

  CycleFlowGraph flow_graph = CycleFlowGraph(before);
  ASSERT_EQ(flow_graph._graph, after_expanded);
}
TEST(GraphOperation, DetermineNumberOfNodeDisjointCycles) {
  Graph before = read_instance_from_file(test_root / "t_009")._graph;
  std::vector<size_t> correct_cycles = {
      2, 2, 2, 1, 3, 3, 3, 3, 3}; // this is hardcoded. Change when t_009 is modified.
  std::vector<size_t> our_cycles(9, 111);

  CycleFlowGraph flow_graph = CycleFlowGraph(before);

  for (size_t idx = 0; idx < 9; idx++) {
    size_t in_idx = 2 * idx, out_idx = 2 * idx + 1;
    our_cycles[idx] = flow_graph.get_max_flow(out_idx, in_idx, 9);
  };
  ASSERT_EQ(correct_cycles, our_cycles);
}
TEST(GraphReductions, ShortcutSingleCycle) {
  Graph before = read_instance_from_file(test_root / "t_009")._graph,
        after_shortcutting =
            read_instance_from_file(test_root / "t_009_shortcutted_single_cycle")._graph;

  auto instance = FvsInstance(before);
  instance.reduction_shortcut_nodes_on_single_cycles();
  instance.reduction_remove_isolated_nodes();
  ASSERT_EQ(instance._graph, after_shortcutting);
}
TEST(GraphReductions, ArticulationPoints) {
  Graph graph = read_instance_from_file(test_root / "t_articulation_point_components")._graph;
  auto cut_nodes = graph.weak_articulation_points();

  // change once t_articulation_point_components changes
  std::vector<size_t> expected_cut_nodes = {0, 3, 4, 9};
  ASSERT_TRUE(std::is_permutation(cut_nodes.begin(), cut_nodes.end(), expected_cut_nodes.begin(),
                                  expected_cut_nodes.end()));
}
TEST(GraphReductions, ArticulationPointsAggreesWithWCC) {
  for (size_t i = 1; i <= 10000; i++) {
    auto num_nodes = 2 + gen() % 20ul;
    auto num_arcs = gen() % (num_nodes * num_nodes / 2);
    spdlog::info("Checking cut vertices with WCC with n={}, m={}, seed={}", num_nodes, num_arcs, i);

    auto instance = algorithms::generate_random_instance(num_nodes, num_arcs, i)._graph;
    instance.symmetrize();

    auto cut_nodes = instance.weak_articulation_points();
    auto [num_ccs, _initial_cc_colors] = instance.weakly_connected_components();
    // find cut vertices by removing them and checking for disconnections
    std::vector<size_t> wcc_cut_nodes;
    for (size_t idx = 0; idx < instance.size(); idx++) {
      auto copy = instance;
      copy.remove_nodes([idx](const auto &node) { return node.index() == idx; });
      auto [cur_num_ccs, _cur_scc_colors] = copy.weakly_connected_components();
      if (cur_num_ccs > num_ccs) {
        wcc_cut_nodes.push_back(idx);
      }
    }

    ASSERT_TRUE(std::is_permutation(cut_nodes.begin(), cut_nodes.end(), wcc_cut_nodes.begin(),
                                    wcc_cut_nodes.end()));
  }
}
TEST(GraphOperations, WCCAggreesWithSCC) {
  for (size_t i = 1; i <= 10000; i++) {
    auto num_nodes = 2 + gen() % 20ul;
    auto num_arcs = gen() % (num_nodes * num_nodes / 2);
    spdlog::info("Checking WCC with SCC with n={}, m={}, seed={}", num_nodes, num_arcs, i);

    auto instance = algorithms::generate_random_instance(num_nodes, num_arcs, i)._graph;
    instance.symmetrize();

    auto [num_ccs, cols_cc] = instance.weakly_connected_components();
    auto [num_sccs, cols_scc] = instance.strongly_connected_components();

    ASSERT_EQ(num_ccs, num_sccs);
    for (size_t idx1 = 0; idx1 < instance.size(); idx1++) {
      for (size_t idx2 = idx1 + 1; idx2 < instance.size(); idx2++) {
        ASSERT_EQ(cols_cc[idx1] == cols_cc[idx2], cols_scc[idx1] == cols_scc[idx2]);
      }
    }
  }
}
TEST(GraphOperations, FindDisjointCyclesAcutallyFindsCycles) {
  for (size_t i = 1; i <= 10000; i++) {
    auto num_nodes = 2 + gen() % 20ul;
    auto num_arcs = gen() % (num_nodes * num_nodes / 2);
    spdlog::info("Checking find cycles gives cycles with n={}, m={}, seed={}", num_nodes, num_arcs,
                 i);

    auto graph = algorithms::generate_random_instance(num_nodes, num_arcs, i)._graph;
    for (auto cycle : graph.greedy_find_disjoint_cycles()) {
      for (size_t cycle_idx = 0; cycle_idx < cycle.size(); cycle_idx++) {
        auto cur_node_idx = cycle[cycle_idx];
        auto next_node_idx = cycle[(cycle_idx + 1) % cycle.size()];
        ASSERT_NE(std::find(graph[cur_node_idx].out_edges().begin(),
                            graph[cur_node_idx].out_edges().end(), next_node_idx),
                  graph[cur_node_idx].out_edges().end())
            << "the found cycle does not exists in the graph";
      }
    }
  }
}
TEST(GraphOperations, FindDisjointCyclesResultsAreDisjoint) {
  for (size_t i = 1; i <= 10000; i++) {
    auto num_nodes = 2 + gen() % 20ul;
    auto num_arcs = gen() % (num_nodes * num_nodes / 2);
    spdlog::info("Checking find cycles gives *disjoint* cycles with n={}, m={}, seed={}", num_nodes,
                 num_arcs, i);

    auto graph = algorithms::generate_random_instance(num_nodes, num_arcs, i)._graph;
    std::vector<size_t> all_nodes;
    for (auto cycle : graph.greedy_find_disjoint_cycles()) {
      std::copy(cycle.begin(), cycle.end(), std::back_inserter(all_nodes));
    }
    std::sort(all_nodes.begin(), all_nodes.end());
    ASSERT_EQ(std::unique(all_nodes.begin(), all_nodes.end()), all_nodes.end())
        << "Cycles were not disjoint";
  }
}
TEST(GraphOperations, FindDisjointCyclesComplementIsAcyclic) {
  for (size_t i = 1; i <= 10000; i++) {
    auto num_nodes = 2 + gen() % 20ul;
    auto num_arcs = gen() % (num_nodes * num_nodes / 2);
    spdlog::info("Checking find cycles complement is acyclic with n={}, m={}, seed={}", num_nodes,
                 num_arcs, i);

    auto graph = algorithms::generate_random_instance(num_nodes, num_arcs, i)._graph;
    for (auto cycle : graph.greedy_find_disjoint_cycles()) {
      for (auto cycle_node_idx : cycle) {
        graph.isolate_node(cycle_node_idx);
      }
    }
    ASSERT_TRUE(graph.is_acyclic()) << "Complement of greedy cycles is not acyclic";
  }
}
