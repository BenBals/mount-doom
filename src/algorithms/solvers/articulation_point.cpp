#include "articulation_point.h"
#include "helpers.h"

namespace algorithms {
ArticulationPointSolver::ArticulationPointSolver(algorithms::Solver *applicableSolver,
                                                 algorithms::Solver *notApplicableSolver)
    : _applicable_solver(applicableSolver), _not_applicable_solver(notApplicableSolver) {}

std::optional<std::vector<size_t>>
ArticulationPointSolver::solve_applicable(FvsInstance &&instance) {
  helpers::assert_and_log(_applicable_solver != nullptr, "Sub solver for applicable case not set");
  return _applicable_solver->solve(std::move(instance));
}
std::optional<std::vector<size_t>>
ArticulationPointSolver::solve_not_applicable(FvsInstance &&instance) {
  helpers::assert_and_log(_not_applicable_solver != nullptr,
                          "Sub solver for non applicable case not set");
  return _not_applicable_solver->solve(std::move(instance));
}

Solver *ArticulationPointSolver::applicable_solver() const { return _applicable_solver; }
void ArticulationPointSolver::applicable_solver(Solver *inner) { _applicable_solver = inner; }
Solver *ArticulationPointSolver::not_applicable_solver() const { return _not_applicable_solver; }
void ArticulationPointSolver::not_applicable_solver(Solver *inner) {
  _not_applicable_solver = inner;
}

std::optional<std::vector<size_t>> ArticulationPointSolver::solve(FvsInstance &&instance) {
  auto cut_nodes = instance._graph.weak_articulation_points();
  if (cut_nodes.empty()) {
    return solve_not_applicable(std::move(instance));
  }

  Graph graph_copy = instance._graph;
  for (const auto node_idx : cut_nodes) {
    graph_copy.isolate_node(node_idx);
  }
  // we now want to find the remaining component, that is the smallest among the components adjacent
  // to exactly one cut node
  size_t num_ccs;
  std::vector<size_t> colors;
  tie(num_ccs, colors) = graph_copy.weakly_connected_components();

  // prepare search
  std::set cut_node_set(cut_nodes.begin(), cut_nodes.end());
  std::vector<std::vector<size_t>> nodes_of_components(num_ccs);
  std::vector<std::set<size_t>> adjacent_cut_nodes(num_ccs);
  std::vector<bool> is_cut_node_components(num_ccs);
  for (size_t idx = 0; idx < colors.size(); idx++) {
    nodes_of_components[colors[idx]].push_back(idx);
    if (cut_node_set.contains(idx)) {
      is_cut_node_components[colors[idx]] = true;
      continue;
    }

    // need to use _graph as graph_copy has edges to cut nodes removed
    for (auto neighbor : instance._graph[idx].out_edges()) {
      if (cut_node_set.contains(neighbor)) {
        adjacent_cut_nodes[colors[idx]].emplace(neighbor);
      }
    }
    for (auto neighbor : instance._graph[idx].in_edges()) {
      if (cut_node_set.contains(neighbor)) {
        adjacent_cut_nodes[colors[idx]].emplace(neighbor);
      }
    }
  }

  // do actual search
  std::optional<std::pair<size_t, size_t>> smallest; // size, color
  for (size_t col = 0; col < num_ccs; col++) {
    if (adjacent_cut_nodes[col].size() != 1)
      continue;

    std::pair<size_t, size_t> current_value = {nodes_of_components[col].size(), col};
    if (!smallest || current_value < smallest.value())
      smallest = {current_value};
  }

  helpers::assert_and_log(smallest.has_value(),
                          "Found no applicable component, despite there needing to be one");
  auto selected_color = smallest->second;
  auto selected_cut_node = *adjacent_cut_nodes[selected_color].begin();

  // create sub instances
  FvsInstance selected_component_without_node = instance;
  selected_component_without_node.drop_partial_solution();
  auto [removed_nodes, permutation] = selected_component_without_node.remove_nodes(
      [selected_color, selected_cut_node, &colors](const auto &node) {
        return node.index() != selected_cut_node && selected_color != colors[node.index()];
      });
  FvsInstance selected_component_with_node = selected_component_without_node;

  auto selected_cut_node_after_udpate = permutation[selected_cut_node];
  selected_component_without_node.isolate_node(selected_cut_node_after_udpate, true);
  selected_component_with_node.shortcut_node(selected_cut_node_after_udpate, true);

  // solve sub instances
  auto solution_without = solve_applicable(std::move(selected_component_without_node));
  if (!solution_without)
    return {};

  selected_component_with_node._upper_bound = solution_without->size() + 1;
  auto solution_with = solve_applicable(std::move(selected_component_with_node));

  // find sub_solution to take
  std::set<size_t> add_set_original_indices;
  if (solution_with) {
    helpers::assert_and_log(solution_with.value().size() <= solution_without->size(),
                            "sub solver gave too big solution");
    add_set_original_indices = std::set(solution_with->begin(), solution_with->end());
  } else {
    add_set_original_indices = std::set(solution_without->begin(), solution_without->end());
    add_set_original_indices.emplace(instance._original_indices[selected_cut_node]);
  }

  for (size_t idx = 0; idx < instance._graph.size(); idx++) {
    if (add_set_original_indices.contains(instance._original_indices[idx])) {
      instance.take_node_into_partial_solution(idx);
    }
  }

  // remove this component from the instance
  instance.remove_nodes([&](const auto &node) { return colors[node.index()] == selected_color; });

  return solve_applicable(std::move(instance));
}
} // namespace algorithms