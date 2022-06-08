#include "fvs_instance.h"
#include "cycle_flow_graph.h"
#include "src/helpers.h"

#include <algorithm>
#include <numeric>
#include <queue>
#include <random>
#include <utility>

namespace datastructures {

FvsInstance::FvsInstance(const Graph &graph) : FvsInstance(graph, "") {}
FvsInstance::FvsInstance(std::string name) : FvsInstance({}, std::move(name)) {}
FvsInstance::FvsInstance(const Graph &graph, std::string name)
    : _graph(graph), _name(std::move(name)), _original_indices(graph.size()),
      _exclusion_markers(graph.size(), false) {
  std::iota(_original_indices.begin(), _original_indices.end(), 1);
}

size_t FvsInstance::create_node(size_t original_index, bool is_excluded) {
  auto new_index = _graph.create_node();
  if (new_index >= _original_indices.size())
    _original_indices.resize(new_index + 1);
  if (new_index >= _exclusion_markers.size())
    _exclusion_markers.resize(new_index + 1);
  _original_indices[new_index] = original_index;
  _exclusion_markers[new_index] = is_excluded;
  return new_index;
}

// SOLUTION

void FvsInstance::set_solution_size(size_t solution_size) { _known_solution_size = solution_size; }

std::optional<size_t> FvsInstance::known_solution_size() const { return _known_solution_size; }

size_t FvsInstance::partial_solution_size() const { return _partial_solution.size(); }

void FvsInstance::drop_partial_solution() {
  helpers::assert_and_log(_upper_bound >= _partial_solution.size(),
                          "Partial solution was bigger than upper bound and forced to drop");
  _upper_bound -= _partial_solution.size();
  _partial_solution.clear();
}

std::set<size_t> FvsInstance::partial_solution_set() const {
  std::set<size_t> output;
  for (auto node : _partial_solution) {
    output.insert(node);
  }
  return output;
}

void FvsInstance::take_node_into_partial_solution(size_t idx) {
  helpers::assert_and_log(!is_node_excluded(idx),
                          "node was excluded from solution but taken into partial solution!");
  isolate_node(idx, true);
  _partial_solution.push_back(_original_indices[idx]);
}

// Exclusion

bool FvsInstance::is_node_excluded(size_t i) const { return _exclusion_markers[i]; }

void FvsInstance::exclude_node_from_partial_solution(size_t idx) {
  _exclusion_markers[idx] = true;

  _neighbors_to_excluded.emplace(idx);
  shortcut_node(idx);
  _neighbors_to_excluded.erase(idx);
}

bool FvsInstance::is_infeasible_by_excluded_nodes() const {
  auto excluded_subgraph = Graph(_graph.size());

  for (const auto &node : _graph._nodes) {
    if (_exclusion_markers[node.index()]) {
      for (auto &neighbor : node.out_edges()) {
        if (_exclusion_markers[neighbor]) {
          excluded_subgraph.add_arc_unsafe(node.index(), neighbor);
        }
      }
    }
  }

  return !excluded_subgraph.is_acyclic();
}

void FvsInstance::isolate_node(size_t idx, bool may_destroy_self_loops) {
  helpers::assert_and_log(may_destroy_self_loops || !_graph.is_arc(idx, idx),
                          "Isolating a node with a self loop");
  _graph.isolate_node(idx);
}
void FvsInstance::shortcut_node(size_t idx, bool may_destroy_self_loops) {
  helpers::assert_and_log(may_destroy_self_loops || !_graph.is_arc(idx, idx),
                          "Shortcutting a node with a self loop");
  if (_neighbors_to_excluded.contains(idx)) {
    for (auto neighbor : _graph[idx].in_edges()) {
      if (!_exclusion_markers[neighbor]) {
        _neighbors_to_excluded.emplace(neighbor);
      }
    }
    for (auto neighbor : _graph[idx].out_edges()) {
      if (!_exclusion_markers[neighbor]) {
        _neighbors_to_excluded.emplace(neighbor);
      }
    }
  }
  _graph.shortcut_node(idx);
}

// MISC

void FvsInstance::permute() {
  std::vector<size_t> perm(_graph.size());
  std::iota(perm.begin(), perm.end(), 0);
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(perm.begin(), perm.end(), g);

  Graph permuted_graph(_graph.size());

  for (const auto &node : _graph._nodes) {
    for (const auto &neighbor : node.out_edges()) {
      permuted_graph.add_arc_unsafe(perm[node.index()], perm[neighbor]);
    }
  }

  _graph = std::move(permuted_graph);
  std::vector<size_t> new_original_indices(_graph.size());
  for (size_t idx = 0; idx < _graph.size(); idx++) {
    new_original_indices[perm[idx]] = _original_indices[idx];
  }
  _original_indices = std::move(new_original_indices);
}

std::string FvsInstance::export_as_graphviz() const {
  std::stringstream out;
  const auto cut_nodes_vec = _graph.weak_articulation_points();
  std::set<size_t> cut_nodes(cut_nodes_vec.begin(), cut_nodes_vec.end());

  out << "digraph " << _name << " { graph [overlap=false]; ";
  for (size_t idx = 0; idx < _graph.size(); idx++) {
    out << "n" << idx << " [label=\"" << _original_indices[idx] << "\"";
    if (cut_nodes.contains(idx)) {
      out << R"( color="blue" penwidth="3.0")";
    }
    out << "];";
  }
  for (size_t idx = 0; idx < _graph.size(); idx++) {
    for (const auto k2_neighbor : _graph[idx].undirected_edges()) {
      if (k2_neighbor < idx)
        out << "n" << idx << " -> "
            << "n" << k2_neighbor << R"( [dir="none" color="gray"];)";
    }
    for (const auto non_k2_neighbor : _graph[idx].out_edges_not_undirected()) {
      out << "n" << idx << " -> "
          << "n" << non_k2_neighbor << R"( [color="red" penwidth="3.0"];)";
    }
  }
  out << "}";
  std::string output = out.str();
  return out.str();
}

// HELPER

std::pair<std::vector<FvsInstance>, std::vector<size_t>>
split_fvs_instance_by_sccs(FvsInstance &&instance) {
  const auto [num_colors, colors] = instance._graph.strongly_connected_components();
  if (num_colors <= 1) {
    auto partial_solution = std::move(instance._partial_solution);
    return {{std::move(instance)}, std::move(partial_solution)};
  }

  std::vector<size_t> new_index(instance._graph.size());
  std::vector<FvsInstance> instances(num_colors, FvsInstance(instance._name));
  for (size_t i = 0; i < colors.size(); i++) {
    new_index[i] = instances[colors[i]].create_node(instance._original_indices[i],
                                                    instance._exclusion_markers[i]);
  }

  for (size_t i = 0; i < colors.size(); i++) {
    for (const auto out_neighbor : instance._graph[i].out_edges()) {
      if (colors[i] == colors[out_neighbor])
        instances[colors[i]]._graph.add_arc_unsafe(new_index[i], new_index[out_neighbor]);
    }
  }
  return {std::move(instances), std::move(instance._partial_solution)};
}
bool FvsInstance::is_solved() const {
  auto copy = *this;
  auto pss = copy.partial_solution_set();
  copy.remove_nodes(
      [&](const auto &node) { return pss.count(copy._original_indices[node.index()]); });

  return copy._graph.is_acyclic();
}

bool FvsInstance::is_vc_instance() {
  return std::all_of(_graph._nodes.begin(), _graph._nodes.end(), [](const auto &node) {
    if (!node.all_edges_not_undirected().empty()) {
      return false;
    }
    return true;
  });
}
size_t FvsInstance::lower_bound() const { return _graph.greedy_count_disjoint_cycles(); }

} // namespace datastructures
