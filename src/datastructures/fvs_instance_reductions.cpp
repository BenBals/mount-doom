#include "cycle_flow_graph.h"
#include "fvs_instance.h"

#include <algorithm>
#include <numeric>
#include <queue>

namespace datastructures {

void FvsInstance::reductions_exhaustively() {
  bool graph_changed;
  // do {
  do {
    graph_changed = false;
    graph_changed |= reduction_shortcut_clique_adjacent_nodes(4);
    graph_changed |= reduction_delete_arcs_between_strongly_connected_components_ignoring_k2();
    collect_self_loops_into_partial_fvs();
    reduction_remove_isolated_nodes();
  } while (!_graph.empty() && graph_changed);
  // } while (reduction_shortcut_nodes_on_single_cycles());
}

size_t FvsInstance::reduction_remove_isolated_nodes() {
  return remove_nodes([](auto &node) { return node.all_edges().empty(); }).first;
}

bool FvsInstance::reduction_shortcut_clique_adjacent_nodes(size_t check_bound) {
  auto is_directed_clique = [this](const Graph::Node::EdgeList &nodes) {
    for (const auto idx : nodes)
      if (_graph[idx].undirected_edges().size() < nodes.size() - 1)
        return false;

    for (auto from : nodes) {
      size_t found = 0;
      for (auto to : _graph[from].undirected_edges()) {
        found += to != from && std::find(nodes.begin(), nodes.end(), to) != nodes.end();
      }
      if (found != nodes.size() - 1)
        return false;
    }

    return true;
  };

  bool graph_changed = false;
  for (size_t idx = 0; idx < _graph.size(); idx++) {
    if (_graph.is_arc(idx, idx) || _graph[idx].all_edges().empty())
      continue;

    if ((_graph[idx].in_edges().size() <= check_bound &&
         is_directed_clique(_graph[idx].in_edges())) ||
        (_graph[idx].out_edges().size() <= check_bound &&
         is_directed_clique(_graph[idx].out_edges()))) {
      graph_changed = true;
      shortcut_node(idx);
    }
  }
  return graph_changed;
}

bool FvsInstance::reduction_delete_arcs_between_strongly_connected_components_ignoring_k2() {
  size_t num_colors;
  std::vector<size_t> colors;
  tie(num_colors, colors) = _graph.strongly_connected_components_ignoring_k2();
  if (num_colors <= 1)
    return false;

  bool changed_graph = false;
  for (size_t idx = 0; idx < _graph.size(); idx++) {
    auto &node = _graph[idx];

    auto pre_deletion_node_out_size = node.all_edges().size();
    _graph.remove_neighbors_of_node_unsafe(
        idx, [&colors, idx](auto const neighbor, const auto edge_direction) {
          return colors[idx] != colors[neighbor.index()] &&
                 edge_direction != Graph::EdgeDirection::UNDIRECTED;
        });
    changed_graph |= pre_deletion_node_out_size != node.all_edges().size();
  }
  return changed_graph;
}

size_t FvsInstance::reduction_shortcut_nodes_on_single_cycles() {
  CycleFlowGraph flow_graph = CycleFlowGraph(_graph);
  size_t num_shortcutted_nodes = 0;

  for (size_t idx = 0; idx < _graph.size(); idx++) {
    size_t in_idx = 2 * idx, out_idx = 2 * idx + 1;
    size_t num_node_disjoint_cycles = flow_graph.get_max_flow(out_idx, in_idx, 2);
    if (num_node_disjoint_cycles <= 1) {
      // this just works
      flow_graph._graph.shortcut_node(in_idx);
      flow_graph._graph.shortcut_node(out_idx);
      shortcut_node(idx);
      if (num_node_disjoint_cycles == 1)
        num_shortcutted_nodes++;
    }
  }

  return num_shortcutted_nodes;
}

void FvsInstance::collect_self_loops_into_partial_fvs() {
  for (size_t idx = 0; idx < _graph.size(); idx++) {
    if (_graph.is_arc(idx, idx)) {
      _partial_solution.push_back(_original_indices[idx]);
      isolate_node(idx, true);
    }
  }
}
} // namespace datastructures
