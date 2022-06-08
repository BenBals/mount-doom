#pragma once

#include "graph.h"
#include "helpers.h"

#include "spdlog/spdlog.h"
#include <bitset>
#include <concepts>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <unordered_set>
#include <vector>

#include "partial_solution.h"

namespace datastructures {

class FvsInstance {
private:
  std::optional<size_t> _known_solution_size;

public:
  Graph _graph;
  std::string _name;
  // original indices of nodes in our partial fvs
  std::vector<size_t> _original_indices;
  PartialSolution _partial_solution;
  std::unordered_set<size_t> _neighbors_to_excluded;
  /**
   * It is guaranteed that a solution strictly smaller than this value exists.
   */
  size_t _upper_bound = SIZE_MAX;

  bool is_node_excluded(size_t i) const;
  /**
   * This function clears the current _partial_solution and adjusts _upper_bound, such that the
   * number of nodes, we can take in this branch, stays constant.
   *
   * This is useful for constructing subinstances, that do not share the already found nodes.
   * @return the current partial solution
   */
  PartialSolution forget_partial_solution();
  size_t partial_solution_size() const;
  void set_solution_size(size_t solution_size);
  std::optional<size_t> known_solution_size() const;

  FvsInstance() = default;
  explicit FvsInstance(const Graph &graph);
  explicit FvsInstance(std::string name);
  FvsInstance(const Graph &graph, std::string name);

  /**
   * Removes edges, that are dominated by other edges
   */
  bool reduction_remove_dominated_edges();
  /**
   * @return true if the graph was changed, false otherwise
   */
  bool reduction_shortcut_clique_adjacent_nodes(size_t nodes = SIZE_MAX);
  /**
   * Please ensure that before each call nodes with self-loops are collected into the fvs.
   * Else they might count towards the shortcutted nodes
   * @return Number of shortcutted nodes
   */
  size_t reduction_shortcut_nodes_on_single_cycles();
  /**
   * @return (number of nodes removed, permutation applied)
   * For more details look into the same named method on Graph
   */
  template <class F>
  requires std::predicate<F, Graph::Node &> std::pair<size_t, std::vector<size_t>>
  remove_nodes(F &&f);
  size_t reduction_remove_isolated_nodes();
  /**
   * @return true if the graph was changed, false otherwise
   */
  bool reduction_delete_arcs_between_strongly_connected_components_ignoring_k2();
  /**
   * @return true if the graph was changed, false otherwise
   */
  bool reduction_fold_degree_2_nodes();
  /**
   * @return true if the graph was changed, false otherwise
   */
  bool reduction_pick_undirected_neighbor_if_undirected_neighborhood_is_superset();
  /**
   * @return true if the graph was changed, false otherwise
   */
  bool reduction_shortcut_directed_in_and_out_degree_1();
  /**
   * Performs the funnel reduction
   * @param fold Whether the funnel should be folded
   * @param check_limit Maximum degree for check
   * @return Whether the graph changed
   */
  bool reduction_funnel(size_t check_limit = SIZE_MAX, bool fold = true);
  bool collect_self_loops_into_partial_fvs();
  void reductions_exhaustively();
  void light_reductions_exhaustively();

  /**
   * Gives a lower bound on the remaining number of nodes to take. If there is at least one cycle,
   * this method yields a lower bound of at least one.
   */
  size_t lower_bound() const;

  size_t create_node(size_t original_index, bool is_excluded);

  void take_node_into_partial_solution(size_t idx);
  void exclude_node_from_partial_solution(size_t idx);
  /**
   * @param idx
   * @param may_destroy_self_loops In *DEBUG* mode, we check that no self loops get removed, when
   * this is set
   */
  void isolate_node(size_t idx, bool may_destroy_self_loops = false);
  /**
   * @param idx
   * @param may_destroy_self_loops In *DEBUG* mode, we check that no self loops get removed, when
   * this is set
   */
  void shortcut_node(size_t idx, bool may_destroy_self_loops = false);
  bool is_infeasible_by_excluded_nodes() const;

  template <class F>
  requires helpers::keying_function<F, size_t> std::optional<size_t>
  get_neighbor_from_excluded_with_highest_by(F &&key_function)
  const;

  // We mark these nodes as *not* being part of any solution to this instance.
  std::vector<bool> _exclusion_markers;

  void permute();

  std::string export_as_graphviz() const;

  friend std::pair<std::vector<FvsInstance>, PartialSolution>
  split_fvs_instance_by_sccs(FvsInstance &&instance);

  bool is_vc_instance();
};

// the FvsInstance, on which this function is called, gets decomposed into its parts and should
// not be used afterwards
std::pair<std::vector<FvsInstance>, PartialSolution>
split_fvs_instance_by_sccs(FvsInstance &&instance);
} // namespace datastructures

#include "fvs_instance_impl.h"
