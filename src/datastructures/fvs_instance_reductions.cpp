#include "cycle_flow_graph.h"
#include "fvs_instance.h"

#include <algorithm>
#include <array>
#include <numeric>
#include <queue>

namespace datastructures {

void FvsInstance::reductions_exhaustively() {
start:
  collect_self_loops_into_partial_fvs();
  if (reduction_remove_dominated_edges())
    goto start;
  if (reduction_shortcut_clique_adjacent_nodes())
    goto start;
  if (reduction_delete_arcs_between_strongly_connected_components_ignoring_k2())
    goto start;
  if (reduction_shortcut_directed_in_and_out_degree_1())
    goto start;
  if (reduction_pick_undirected_neighbor_if_undirected_neighborhood_is_superset())
    goto start;
  if (reduction_fold_degree_2_nodes())
    goto start;
  if (reduction_funnel())
    goto start;

  // in the end remove isolated nodes
  reduction_remove_isolated_nodes();
}

void FvsInstance::light_reductions_exhaustively() {
  do {
    collect_self_loops_into_partial_fvs();
    while (reduction_shortcut_clique_adjacent_nodes())
      ;
  } while (reduction_delete_arcs_between_strongly_connected_components_ignoring_k2());
  reduction_remove_isolated_nodes();
}

size_t FvsInstance::reduction_remove_isolated_nodes() {
  return remove_nodes([](auto &node) { return node.all_edges().empty(); }).first;
}

bool FvsInstance::reduction_remove_dominated_edges() {
  bool graph_changed = false;
  std::vector<size_t> direct_out_nodes, out_edges_to_delete;
  for (auto &node : _graph._nodes) {
    out_edges_to_delete.clear();
    direct_out_nodes.clear();

    std::copy(node.out_edges().begin(), node.out_edges().end(),
              std::back_inserter(direct_out_nodes));
    std::sort(direct_out_nodes.begin(), direct_out_nodes.end());

    // check out edges
    for (auto out_node : node.out_edges_not_undirected()) {
      bool may_delete = true;
      for (auto dist_2_out_node : _graph[out_node].out_edges_not_undirected()) {
        if (!std::binary_search(direct_out_nodes.begin(), direct_out_nodes.end(),
                                dist_2_out_node)) {
          may_delete = false;
          break;
        }
      }

      if (may_delete) {
        out_edges_to_delete.emplace_back(out_node);
      }
    }
    // we don't need to check in edges, since when this condition is satisfied for an in edge, it is
    // also satisfied for this edge looked from the other node, where this edge is an out edge

    if (!out_edges_to_delete.empty()) {
      for (auto out_node : out_edges_to_delete) {
        _graph.remove_arc_unsafe(node.index(), out_node);
      }
      graph_changed = true;
    }
  }
  return graph_changed;
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

bool FvsInstance::reduction_fold_degree_2_nodes() {
  bool graph_changed = false;

  for (const auto &node : _graph._nodes) {
    if (node.undirected_edges().size() != 2 || !node.all_edges_not_undirected().empty())
      continue;

    std::array<size_t, 2> neighbors{};
    std::copy(node.undirected_edges().begin(), node.undirected_edges().end(), neighbors.begin());

    // we want neighbors[0] to only have undirected edges
    if (!_graph[neighbors[0]].all_edges_not_undirected().empty()) {
      if (!_graph[neighbors[1]].all_edges_not_undirected().empty())
        // in this case both have not undirected edges
        continue;
      std::swap(neighbors[0], neighbors[1]);
    }

    // ensure that the neighbors are not adjacent
    if (_graph.is_arc(neighbors[0], neighbors[1]))
      // the other direction is also fine, since neighbor[0] only has undirected edges
      continue;
    // ensure, that none of the removed vertices has a self loop
    if (_graph.is_arc(node.index(), node.index()) || _graph.is_arc(neighbors[0], neighbors[0]))
      continue;

    // remember unification
    graph_changed = true;
    auto unified_node_idx = neighbors[1];
    _partial_solution.add_degree_2_unification(
        _original_indices[node.index()], _original_indices[neighbors[0]],
        _original_indices[neighbors[1]], _original_indices[unified_node_idx]);

    // actually unify neighbors
    for (auto dist2_neighbor : _graph[neighbors[0]].undirected_edges()) {
      if (dist2_neighbor == node.index())
        continue;

      _graph.add_arc(dist2_neighbor, unified_node_idx);
      _graph.add_arc(unified_node_idx, dist2_neighbor);
    }

    // "remove" non unified nodes
    isolate_node(node.index());
    isolate_node(neighbors[0]);
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

bool FvsInstance::reduction_pick_undirected_neighbor_if_undirected_neighborhood_is_superset() {
  bool changed = false;

  for (size_t idx = 0; idx < _graph.size(); idx++) {
    const auto &node = _graph[idx];
    const auto all_neighbors = node.all_edges();

    bool found_dominating_node;
    do {
      found_dominating_node = false;
      for (auto undirected_neighbor : node.undirected_edges()) {
        if (_graph[undirected_neighbor].undirected_edges().size() < all_neighbors.size()) {
          // can't be adjacent to all all_neighbors
          continue;
        }

        size_t found = 0;
        for (auto undirected_neighbor_neighbor : _graph[undirected_neighbor].undirected_edges()) {

          // this also counts correctly if undirected_neighbor or idx have self loops
          found += undirected_neighbor_neighbor != undirected_neighbor &&
                   undirected_neighbor_neighbor != idx &&
                   std::find(all_neighbors.begin(), all_neighbors.end(),
                             undirected_neighbor_neighbor) != all_neighbors.end();
        }
        if (found == all_neighbors.size() - 1) {
          take_node_into_partial_solution(undirected_neighbor);
          changed = true;
          found_dominating_node = true;
          break; // graph changed so, we can't iterate further
        }
      }
    } while (found_dominating_node);
  }

  return changed;
}

bool FvsInstance::reduction_shortcut_directed_in_and_out_degree_1() {
  bool changed = false;
  for (size_t idx = 0; idx < _graph.size(); idx++) {
    auto v = _graph[idx];
    if (v.in_edges_not_undirected().size() == 1 && v.out_edges_not_undirected().size() == 1) {
      auto u = _graph[*v.in_edges_not_undirected().begin()];
      auto w = _graph[*v.out_edges_not_undirected().begin()];

      auto neighbors_u = u.undirected_edges();
      auto neighbors_w = w.undirected_edges();

      bool found_all = true;
      for (auto undirected_neighbor : v.undirected_edges()) {
        found_all &= std::find(neighbors_u.begin(), neighbors_u.end(), undirected_neighbor) !=
                         neighbors_u.end() ||
                     std::find(neighbors_w.begin(), neighbors_w.end(), undirected_neighbor) !=
                         neighbors_w.end();
      }

      if (found_all) {
        _graph.remove_arc_unsafe(u.index(), v.index());
        _graph.remove_arc_unsafe(v.index(), w.index());
        _graph.add_arc(u.index(), w.index());
        changed = true;
      }
    }
  }

  return changed;
}

bool FvsInstance::reduction_funnel(size_t check_limit, bool fold) {
  auto is_directed_clique = [this](const auto &nodes) {
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
    const auto &node = _graph[idx];

    if (!node.all_edges_not_undirected().empty() || node.undirected_edges().size() > check_limit ||
        _graph.is_arc(idx, idx))
      continue;

    std::vector<size_t> clique_to_check;
    for (const auto funnel_candidate : node.undirected_edges()) {
      if (_graph.is_arc(funnel_candidate, funnel_candidate))
        continue;

      clique_to_check.clear();
      std::copy_if(node.undirected_edges().begin(), node.undirected_edges().end(),
                   std::back_inserter(clique_to_check), [funnel_candidate](const auto &neighbor) {
                     return neighbor != funnel_candidate;
                   });
      if (!is_directed_clique(clique_to_check))
        continue;

      // take common undirected neighbors
      // this needs to be saved temporarily as changing the graph invalidates iterators
      std::vector<size_t> common_neighbors;
      for (const auto &in_clique : clique_to_check) {
        if (std::find(_graph[in_clique].undirected_edges().begin(),
                      _graph[in_clique].undirected_edges().end(),
                      funnel_candidate) != _graph[in_clique].undirected_edges().end()) {
          common_neighbors.push_back(in_clique);
          graph_changed = true;
        }
      }
      for (auto to_take : common_neighbors) {
        take_node_into_partial_solution(to_take);
      }

      if (fold && _graph[funnel_candidate].all_edges_not_undirected().empty()) {
        // clear common neighbors
        std::erase_if(clique_to_check, [&](const auto &clique_idx) {
          return std::find(common_neighbors.begin(), common_neighbors.end(), clique_idx) !=
                 common_neighbors.end();
        });

        for (auto clique_idx : clique_to_check) {
          // _graph[funnel_candidate].undirected_edges() has common neighbors removed, since they
          // are removed by take_node_into_partial_solution
          for (auto dist_2_neighbor : _graph[funnel_candidate].undirected_edges()) {
            if (dist_2_neighbor == idx)
              continue;

            _graph.add_arc(clique_idx, dist_2_neighbor);
            _graph.add_arc(dist_2_neighbor, clique_idx);
          }
        }

        isolate_node(funnel_candidate);
        isolate_node(idx);

        std::transform(clique_to_check.begin(), clique_to_check.end(), clique_to_check.begin(),
                       [this](auto &clique_idx) { return _original_indices[clique_idx]; });
        _partial_solution.add_funnel_expansion(std::move(clique_to_check),
                                               _original_indices[funnel_candidate],
                                               _original_indices[idx]);
      }

      if (fold || !common_neighbors.empty())
        break; // need to break since we modified the graph
    }
  }
  return graph_changed;
}
bool FvsInstance::collect_self_loops_into_partial_fvs() {
  bool changed = false;
  for (size_t idx = 0; idx < _graph.size(); idx++) {
    if (_graph.is_arc(idx, idx)) {
      _partial_solution.add_node(_original_indices[idx]);
      isolate_node(idx, true);
      changed = true;
    }
  }
  return changed;
}
} // namespace datastructures
