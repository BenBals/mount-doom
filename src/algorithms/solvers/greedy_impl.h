#pragma once

#include "greedy.h"
#include <map>
#include <queue>

namespace algorithms {
template <class F>
requires node_key_function<F>
MaxByKeyHeuristicSolver<F>::MaxByKeyHeuristicSolver(F &&keying_function)
    : NodeKeyed<F>(std::move(keying_function)) {}

template <class F>
requires node_key_function<F>
MaxByKeyHeuristicSolver<F>::MaxByKeyHeuristicSolver(F &&keying_function, bool use_two_one_swaps,
                                                    size_t reduce_after_taking_n)
    : NodeKeyed<F>(std::move(keying_function)), _use_two_one_swaps(use_two_one_swaps),
      _reduce_after_taking_n(reduce_after_taking_n) {}

template <class F>
requires node_key_function<F> std::optional<std::pair<size_t, std::vector<size_t>>>
MaxByKeyHeuristicSolver<F>::find_next_cycle(size_t start_node, const Graph &graph) {
  auto [num_components, components] = graph.strongly_connected_components();
  if (num_components == graph.size())
    return {};

  std::vector<size_t> component_sizes(num_components);
  for (size_t node_index = 0; node_index < graph.size(); node_index++)
    component_sizes[components[node_index]]++;

  for (size_t node_index = start_node; node_index < graph.size(); node_index++) {
    if (component_sizes[components[node_index]] == 1)
      continue;

    auto cycle_opt = graph.find_cycle_with_node(node_index, cycle_finding_data);
    helpers::assert_and_log(cycle_opt.has_value(),
                            "node was in SCC of size 2, but lies on no cycle");
    return {{node_index, cycle_opt.value()}};
  }
  return {};
}

template <class F>
requires node_key_function<F> std::vector<size_t>
MaxByKeyHeuristicSolver<F>::greedily_find_initial_solution(FvsInstance &&instance) {
  // map from original_index to node_index in initial reduced_instance
  std::map<size_t, size_t> original_index_to_reduced_node_index;
  for (size_t node_index = 0; node_index < instance._graph.size(); node_index++) {
    original_index_to_reduced_node_index[instance._original_indices[node_index]] = node_index;
  }

  std::priority_queue<std::pair<node_key_function_value<F>, size_t>> priority_queue;
  for (size_t node_index = 0; node_index < instance._graph.size(); node_index++) {
    priority_queue.emplace(this->get_node_key(node_index, instance), node_index);
  }

  std::vector<size_t> unfiltered_solution;
  size_t round = 0;
  auto current_cycle_opt = find_next_cycle(0, instance._graph);
  while (current_cycle_opt) {
    helpers::assert_and_log(!priority_queue.empty(),
                            "graph is not acyclic, but no nodes are remaining");
    const auto [key, node_index] = priority_queue.top();
    priority_queue.pop();

    // values might have changed to make sure, that they are still accurate
    auto current_key = this->get_node_key(node_index, instance);
    if (current_key != key) {
      priority_queue.push({current_key, node_index});
      continue;
    }

    const auto &node = instance._graph[node_index];
    if (node.all_edges().empty())
      continue;

    round++;
    instance.take_node_into_partial_solution(node_index);
    if (!helpers::is_timeout_hit() && round % _reduce_after_taking_n == 0) {
      instance.reductions_exhaustively();

      // reinitialize helper datastructures
      original_index_to_reduced_node_index.clear();
      for (size_t node_index2 = 0; node_index2 < instance._graph.size(); node_index2++) {
        original_index_to_reduced_node_index[instance._original_indices[node_index2]] = node_index2;
      }
      priority_queue = {};
      for (size_t node_index2 = 0; node_index2 < instance._graph.size(); node_index2++) {
        priority_queue.emplace(this->get_node_key(node_index2, instance), node_index2);
      }

      current_cycle_opt = find_next_cycle(0, instance._graph);
    }

    for (auto new_node_original_index : instance._partial_solution) {
      unfiltered_solution.push_back(new_node_original_index);
      auto new_node_index = original_index_to_reduced_node_index[new_node_original_index];

      if (current_cycle_opt) {
        auto [first_cycle_node, current_cycle] = current_cycle_opt.value();

        bool intersects_current_cycle = std::find(current_cycle.begin(), current_cycle.end(),
                                                  new_node_index) != current_cycle.end();
        // cycle is hit, so find a new cycle
        if (intersects_current_cycle)
          current_cycle_opt = find_next_cycle(first_cycle_node, instance._graph);
      }
    }
    instance.drop_partial_solution();
  }

  return unfiltered_solution;
}

template <class F>
requires node_key_function<F> std::vector<size_t>
MaxByKeyHeuristicSolver<F>::make_solution_minimal(const FvsInstance &instance,
                                                  std::vector<size_t> unfiltered_solution) {
  std::vector<size_t> filtered_solution;
  // map from original_index to node_index in initial reduced_instance
  std::map<size_t, size_t> original_index_to_initially_reduced_node_index;
  for (size_t node_index = 0; node_index < instance._graph.size(); node_index++) {
    original_index_to_initially_reduced_node_index[instance._original_indices[node_index]] =
        node_index;
  }

  // first isolate all nodes in current solution
  std::set<size_t> current_solution_indices;
  auto graph_copy = instance._graph;
  for (auto original_index : unfiltered_solution) {
    auto node_index = original_index_to_initially_reduced_node_index[original_index];
    graph_copy.isolate_node(node_index);
    current_solution_indices.emplace(node_index);
  }

  helpers::assert_and_log(graph_copy.is_acyclic(),
                          "The found dfvs does not make the graph acyclic");

  // filter solution to not take irrelevant nodes
  // i.e. construct a minimal (not minimum) DFVS
  while (!unfiltered_solution.empty() && !helpers::is_termination_requested()) {
    auto current_original_index = unfiltered_solution.back();
    auto current_index = original_index_to_initially_reduced_node_index[current_original_index];
    unfiltered_solution.pop_back();
    current_solution_indices.erase(current_index);

    for (auto in_node_index : instance._graph[current_index].in_edges()) {
      if (!current_solution_indices.contains(in_node_index))
        graph_copy.add_arc(in_node_index, current_index);
    }
    for (auto out_node_index : instance._graph[current_index].out_edges()) {
      if (!current_solution_indices.contains(out_node_index))
        graph_copy.add_arc(current_index, out_node_index);
    }

    if (!graph_copy.find_cycle_with_node(current_index, cycle_finding_data))
      // there is no cycle with this node, so this node does not have to be in the dfvs
      // we need to leave the graph as is, as we actually added this node back into the graph
      continue;

    // this has to be in dfvs
    filtered_solution.push_back(current_original_index);
    current_solution_indices.emplace(current_index);
    graph_copy.isolate_node(current_index);
  }

  // unfiltered solution might not be empty, after termination is hit
  std::copy(unfiltered_solution.begin(), unfiltered_solution.end(),
            std::back_inserter(filtered_solution));

  return filtered_solution;
}

template <class F>
requires node_key_function<F> std::optional<std::vector<size_t>>
MaxByKeyHeuristicSolver<F>::solve(FvsInstance &&instance) {
  const auto initial_upper_bound = instance._upper_bound;

  auto reduced_instance = std::move(instance);
  reduced_instance.reductions_exhaustively(); // this removes all self loops
  std::vector<size_t> final_solution = std::move(reduced_instance._partial_solution);
  reduced_instance.drop_partial_solution();

  auto reduced_instance_copy = reduced_instance;
  auto unfiltered_solution = greedily_find_initial_solution(std::move(reduced_instance));
  auto filtered_solution_minimal =
      make_solution_minimal(reduced_instance_copy, unfiltered_solution);
  auto filtered_solution =
      _use_two_one_swaps
          ? iterative_small_compression(reduced_instance_copy, filtered_solution_minimal)
          : filtered_solution_minimal;

  // rewrite node indices to those used in the original graph
  std::move(filtered_solution.begin(), filtered_solution.end(), std::back_inserter(final_solution));

  if (final_solution.size() < initial_upper_bound)
    return {final_solution};
  return {};
}

template <class F>
requires node_key_function<F> std::vector<size_t>
MaxByKeyHeuristicSolver<F>::iterative_small_compression(const FvsInstance &instance,
                                                        std::vector<size_t> &unfiltered_solution) {
  // just a few declarations, nothing to see here.
  std::vector<size_t> filtered_solution;
  std::set<size_t> unfiltered(unfiltered_solution.begin(), unfiltered_solution.end());
  for (size_t i = 0; i < instance._graph.size(); i++) {
    // set vertices in the solution to the vertices they have in the current graph
    if (unfiltered.contains(instance._original_indices[i]))
      filtered_solution.push_back(i);
  }

  std::vector<bool> in_solution(instance._graph.size(), false);
  for (auto idx : filtered_solution)
    in_solution[idx] = true;

  std::vector<std::vector<size_t>> cycle_separators;
  std::vector<size_t> on_path;
  std::vector<size_t> visited;
  std::vector<size_t> time_stamp;
  std::vector<size_t> topo_sorted;
  std::vector<size_t> order;
  size_t time;

  bool found_improvement = true;
  while (found_improvement && !helpers::is_termination_requested()) {
    found_improvement = false;

    // a topological sorting which only inserts nodes on a path to the start_node
    std::function<void(size_t, size_t)> inv_topo_sort = [&](size_t start_node,
                                                            size_t current_node) {
      bool found_start = (current_node == start_node);
      visited[current_node] = true;
      time_stamp[current_node] = time;
      for (auto out_nei_idx : instance._graph[current_node].out_edges()) {
        if (out_nei_idx == start_node)
          found_start = true;
        if (!in_solution[out_nei_idx]) {
          if (!visited[out_nei_idx] || time_stamp[out_nei_idx] != time) {
            inv_topo_sort(start_node, out_nei_idx);
          }
          found_start |= (visited[out_nei_idx] == 2);
        }
      }
      if (found_start)
        topo_sorted.push_back(current_node);
      // if we found start, value is 2, else 1
      visited[current_node] += found_start;
      return found_start;
    };

    cycle_separators.assign(filtered_solution.size(), {});
    on_path.assign(instance._graph.size(), 0);
    visited.assign(instance._graph.size(), 0);
    time_stamp.assign(instance._graph.size(), 0);

    for (size_t cycle_idx = 0;
         cycle_idx < filtered_solution.size() && !helpers::is_termination_requested();
         cycle_idx++) {
      time = 2 * cycle_idx + 1;
      size_t cycle_node = filtered_solution[cycle_idx];

      // calculate the top.sort of all nodes on any cycle to and from cycle_node
      topo_sorted.assign({});
      inv_topo_sort(cycle_node, cycle_node);
      std::reverse(topo_sorted.begin(), topo_sorted.end());

      time++;
      size_t visited_nodes = 1;
      time_stamp[cycle_node] = time;

      // calculate which nodes are on each cycle from and to cycle node (and are thus feasible to
      // replace cycle_node)
      for (auto it = topo_sorted.begin(); it != topo_sorted.end(); it++) {
        if (helpers::is_termination_requested())
          goto early_return;
        size_t node = *it;
        helpers::assert_and_log(time_stamp[node] >= time - 1, "time to low");
        if (visited_nodes == 1) {
          // node is a separator except it is the starting node
          if (node != cycle_node) {
            cycle_separators[cycle_idx].push_back(node);
          }
        }
        visited_nodes--;
        for (size_t out_nei_idx : instance._graph[node].out_edges()) {
          // only use nodes that are on cycles from cycle_node
          if (visited[out_nei_idx] == 2 && time_stamp[out_nei_idx] >= time - 1) {
            if (time_stamp[out_nei_idx] != time) {
              time_stamp[out_nei_idx] = time;
              on_path[out_nei_idx] = false;
            }
            if (!on_path[out_nei_idx]) {
              visited_nodes++;
              on_path[out_nei_idx] = true;
            }
          }
        }
      }
    }
    // we found all separators

    /*
     * Let's call the potential replacement nodes candidates.
     * The above calculation does not respect that if we remove a second node from the solution, a
     * cycle can use this second node as well. Therefore, the replacement node must also hit paths
     * first_node~>second_node or vice versa, s.t. no cycle can use first_node and second_node.
     *
     * In this implementation, the first node (call it v here) will be fixed and for each other node
     * u in the graph the first candidate (in order of the top. sorting) which can reach u without
     * using other candidates is determined. Any later candidate cannot be taken since there is a
     * path v~>u. We assume that u~>v exists (and some path of them must exist if some candidates
     * are the same for v and u). Afterwards, we check each candidate of u if it is a candidate for
     * v, and it is early enough in the top. sorting (s.t. no v~>u path exists).
     */

    visited.assign(instance._graph.size(), false);
    time_stamp.assign(instance._graph.size(), 0);
    std::queue<size_t> queue;

    for (size_t first_excluded_idx = 0; first_excluded_idx < filtered_solution.size() &&
                                        !found_improvement && !helpers::is_termination_requested();
         first_excluded_idx++) {
      if (cycle_separators[first_excluded_idx].empty())
        // this node cannot be replaced by any other not currently in the solution
        continue;
      time = first_excluded_idx + 1;
      size_t first_excluded_node = filtered_solution[first_excluded_idx];

      // which is the first separator that has a path to the node in the unfiltered solution.
      std::vector<size_t> first_separator(instance._graph.size(), SIZE_MAX);

      // bfs to find all reachable nodes when separators are excluded (i.e. those cannot be excluded
      // from the solution with first_excluded_node)
      helpers::assert_and_log(queue.empty(), "Queue should have been emptied before");
      queue.push(first_excluded_node);
      visited[first_excluded_node] = true;
      time_stamp[first_excluded_node] = time;
      // exclude separators from bfs
      for (auto sep : cycle_separators[first_excluded_idx]) {
        visited[sep] = true;
        time_stamp[sep] = time;
      }
      // find all nodes that can still be reached from first_node, i.e. those cannot be compressed
      while (!queue.empty()) {
        if (helpers::is_termination_requested())
          goto early_return;
        auto next = queue.front();
        queue.pop();
        for (auto out_nei_idx : instance._graph[next].out_edges()) {
          if (time_stamp[out_nei_idx] != time || !visited[out_nei_idx]) {
            if (!in_solution[out_nei_idx])
              queue.push(out_nei_idx);
            visited[out_nei_idx] = true;
            time_stamp[out_nei_idx] = time;
          }
        }
      }

      // inv_cycle_sep is sorted s.t. for i < j, element at index i comes before element at index j
      // in the topological sorting
      for (auto current_separator : cycle_separators[first_excluded_idx]) {
        if (helpers::is_termination_requested())
          goto early_return;
        if (found_improvement)
          break;
        queue.push(current_separator);
        while (!queue.empty()) {
          auto next = queue.front();
          queue.pop();
          for (auto out_nei_idx : instance._graph[next].out_edges()) {
            // if the node was not found before,
            // i.e. it can't be reached from cycle_node without using a separator node
            if (time_stamp[out_nei_idx] != time || !visited[out_nei_idx]) {
              queue.push(out_nei_idx);
              first_separator[out_nei_idx] = current_separator;
              visited[out_nei_idx] = true;
              time_stamp[out_nei_idx] = time;
            }
          }
        }
      }

      order.assign(instance._graph.size(), SIZE_MAX);
      for (size_t i = 0; i < cycle_separators[first_excluded_idx].size(); i++)
        order[cycle_separators[first_excluded_idx][i]] = i;

      for (size_t solution_idx = 0; solution_idx < filtered_solution.size() && !found_improvement &&
                                    !helpers::is_termination_requested();
           solution_idx++) {
        size_t solution_node = filtered_solution[solution_idx];
        if (first_excluded_idx == solution_idx ||
            // if node was not reachable from a separator in this iteration
            (first_separator[solution_node] == SIZE_MAX || time_stamp[solution_node] != time))
          continue;
        for (size_t solution_separator_node : cycle_separators[solution_idx]) {
          // if the sol_sep_node is a separator for the first node (i.e. not SIZE_MAX)
          // and it is before the first separator node from which there is a direct path to solution
          // node (i.e. the sol_sep_node prevents paths first->first, sol_node->sol_node,
          // first->sol_node. By Chen's idea, this node is a 2-1-compression-node.
          if (order[solution_separator_node] <= order[first_separator[solution_node]]) {
            // the separator separates all cycles of first_excluded_idx and solution_node and
            // paths from first_excluded_idx to solution_node this separator is a hero
            found_improvement = true;
            auto old_filtered_solution = filtered_solution; // for better debug possibilties
            filtered_solution[solution_idx] = solution_separator_node;
            // O(1) deletion
            std::swap(filtered_solution[first_excluded_idx], filtered_solution.back());
            filtered_solution.pop_back();

            // as long as tests are not available, please leave this line here
            bool solved = instance._graph.is_acyclic_without_nodes(filtered_solution);
            helpers::assert_and_log(solved, "The compression makes the solution not acyclic");

            in_solution[solution_node] = false;
            in_solution[first_excluded_node] = false;
            in_solution[solution_separator_node] = true;
            break;
          }
        }
      }
    }
  }
early_return:
  std::transform(filtered_solution.begin(), filtered_solution.end(), filtered_solution.begin(),
                 [&](auto idx) { return instance._original_indices[idx]; });
  return filtered_solution;
}

} // namespace algorithms
