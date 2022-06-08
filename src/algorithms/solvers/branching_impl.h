#pragma once

#include "branching.h"
#include "helpers.h"

namespace algorithms {
template <class F>
  requires node_key_function<F>
MaxByKeyBranchingStrategy<F>::MaxByKeyBranchingStrategy(F &&key_function)
    : NodeKeyed<F>(std::move(key_function)) {}

template <class F>
  requires node_key_function<F>
std::optional<std::vector<FvsInstance>>
MaxByKeyBranchingStrategy<F>::branching_instances(FvsInstance &&instance) {
  helpers::assert_and_log(!instance._graph.empty(), "no node left to branch on!");
  auto &node_to_branch_on = *std::max_element(
      instance._graph._nodes.begin(), instance._graph._nodes.end(),
      [&instance, this](const auto &node1, const auto &node2) {
        if (instance.is_node_excluded(node1.index()) != instance.is_node_excluded(node2.index())) {
          return instance.is_node_excluded(node1.index());
        }
        auto key1 = this->get_node_key(node1.index(), instance),
             key2 = this->get_node_key(node2.index(), instance);
        return key1 < key2 || (key1 == key2 && node1.index() < node2.index());
      });
  helpers::assert_and_log(!instance.is_node_excluded(node_to_branch_on.index()),
                          "no excluded node left to branch on!");

  auto instance_take_node = instance;
  auto instance_exclude_node = std::move(instance);

  instance_take_node.take_node_into_partial_solution(node_to_branch_on.index());
  instance_exclude_node.exclude_node_from_partial_solution(node_to_branch_on.index());

  if (instance_exclude_node.is_infeasible_by_excluded_nodes()) {
    return {{std::move(instance_take_node)}};
  }
  return {{std::move(instance_exclude_node), std::move(instance_take_node)}};
}

template <class F>
  requires node_key_function<F>
K2BranchingStrategy<F>::K2BranchingStrategy(F &&key_function)
    : NodeKeyed<F>(std::move(key_function)) {}

template <class F>
  requires node_key_function<F>
std::optional<std::vector<FvsInstance>>
K2BranchingStrategy<F>::branching_instances(FvsInstance &&instance) {
  struct K2Option {
    node_key_function_value<F> key;
    size_t node_idx;
    bool node_excluded, neighbor_excluded;
  };
  std::optional<K2Option> best;

  for (const auto &node : instance._graph._nodes) {
    for (const auto neighbor : node.undirected_edges()) {
      if (node.index() == neighbor)
        continue;
      bool node_excluded = instance.is_node_excluded(node.index());
      bool neighbor_excluded = instance.is_node_excluded(neighbor);

      if (node_excluded && neighbor_excluded)
        continue;

      // found k2
      auto cur_key = this->get_node_key(node.index(), instance);
      if (!best || cur_key > best.value().key) {
        best = {{
            .key = cur_key,
            .node_idx = node.index(),
            .node_excluded = node_excluded,
            .neighbor_excluded = neighbor_excluded,
        }};
      }
    }
  }
  if (!best)
    return {};
  auto data = best.value();

  std::vector<FvsInstance> branching_instances;
  if (!data.node_excluded) {
    auto new_instance = instance;
    new_instance.take_node_into_partial_solution(data.node_idx);
    branching_instances.push_back(new_instance);
  }
  if (!data.neighbor_excluded) {
    auto new_instance = std::move(instance);
    new_instance.exclude_node_from_partial_solution(data.node_idx);
    branching_instances.push_back(new_instance);
  }

  spdlog::debug("\t\t\t\tbranching on k2 with {} options", branching_instances.size());
  return {branching_instances};
}

template <class F>
  requires node_key_function<F>
NonK2BranchingStrategy<F>::NonK2BranchingStrategy(F &&key_function)
    : NodeKeyed<F>(std::move(key_function)) {}

template <class F>
  requires node_key_function<F>
std::optional<std::vector<FvsInstance>>
NonK2BranchingStrategy<F>::branching_instances(FvsInstance &&instance) {
  std::optional<
      std::pair<std::invoke_result_t<F, size_t, const datastructures::FvsInstance &>, size_t>>
      best;

  for (auto &node : instance._graph._nodes) {
    if (instance.is_node_excluded(node.index()))
      continue;

    bool is_only_adjacent_to_k2 =
        node.all_edges_not_undirected().empty() &&
        std::find(node.undirected_edges().begin(), node.undirected_edges().end(), node.index()) ==
            node.undirected_edges().end();
    if (is_only_adjacent_to_k2)
      continue;

    auto current_value = std::pair(this->get_node_key(node.index(), instance), node.index());
    if (!best || current_value > best.value()) {
      best = {current_value};
    }
  }
  if (!best)
    return {};
  auto data = best.value();

  auto include_instance = instance;
  include_instance.take_node_into_partial_solution(data.second);

  auto exclude_instance = std::move(instance);
  exclude_instance.exclude_node_from_partial_solution(data.second);

  return {{std::move(exclude_instance), std::move(include_instance)}};
}

template <class F>
  requires node_key_function<F>
SmallCycleBranchingStrategy<F>::SmallCycleBranchingStrategy(F &&key_function)
    : NodeKeyed<F>(std::move(key_function)) {}

template <class F>
  requires node_key_function<F>
std::optional<std::vector<FvsInstance>>
SmallCycleBranchingStrategy<F>::branching_instances(FvsInstance &&instance) {
  std::vector<FvsInstance> branching_vec;

  for (size_t node_index = 0; node_index < instance._graph.size();
       node_index++) { // we might think about randomizing the order here.
    auto result = instance._graph.find_limited_cycle_with_node(node_index, cycle_finding_data, 3);
    if (result.has_value()) {
      auto data = result.value();
      std::sort(data.begin(), data.end(), [this, &instance](size_t node1_idx, size_t node2_idx) {
        return this->get_node_key(node1_idx, instance) >
               this->get_node_key(node2_idx, instance); // higher value comes first
      });
      for (size_t i = 0; i < data.size() - 1; i++) {
        auto new_instance = instance;
        new_instance.take_node_into_partial_solution(data[i]);
        for (size_t j = 0; j < i; j++) {
          new_instance.exclude_node_from_partial_solution(data[j]);
        }
        branching_vec.push_back(std::move(new_instance));
      }
      auto last_instance = std::move(instance);
      last_instance.take_node_into_partial_solution(data.back());
      for (size_t j = 0; j < data.size() - 1; j++) {
        last_instance.exclude_node_from_partial_solution(data[j]);
      }
      branching_vec.push_back(std::move(last_instance));

      return {branching_vec};
    }
  }

  return {};
}

template <class F>
  requires node_key_function<F>
MaxNeighborByKeyBranchingStrategy<F>::MaxNeighborByKeyBranchingStrategy(F &&key_function)
    : NodeKeyed<F>(std::move(key_function)) {}
template <class F>
  requires node_key_function<F>
std::optional<std::vector<FvsInstance>>
MaxNeighborByKeyBranchingStrategy<F>::branching_instances(FvsInstance &&instance) {
  // if with current excluded, excluding another node creates a cycle on the excluded nodes,
  // we can safely put it in solution set and try again
  // otherwise, we need to consider both options
  auto branching_candidate = instance.get_neighbor_from_excluded_with_highest_by(
      [&, this](size_t idx) { return this->get_node_key(idx, instance); });
  if (!branching_candidate)
    return {};

  auto node_idx = branching_candidate.value();

  auto node_excluded_instance = instance;
  node_excluded_instance.exclude_node_from_partial_solution(node_idx);

  auto node_included_instance = std::move(instance);
  node_included_instance.take_node_into_partial_solution(node_idx);

  if (node_excluded_instance.is_infeasible_by_excluded_nodes()) {
    return {{std::move(node_included_instance)}};
  }
  return {{std::move(node_excluded_instance), std::move(node_included_instance)}};
}

} // namespace algorithms