#pragma once

namespace datastructures {
template <class F>
requires helpers::keying_function<F, size_t> std::optional<size_t>
FvsInstance::get_neighbor_from_excluded_with_highest_by(F &&key_function)
const {
  std::optional<std::pair<decltype(key_function(0)), size_t>> best;

  for (auto node_idx : _neighbors_to_excluded) {
    helpers::assert_and_log(!_exclusion_markers[node_idx],
                            "A node in the neighbor set to excluded nodes must not be excluded!");

    const auto cur_value = key_function(node_idx);
    if (!best || cur_value > best.value().first) {
      best = {cur_value, node_idx};
    }
  }

  if (best) {
    return {best->second};
  }
  return {};
}

template <class F>
requires std::predicate<F, Graph::Node &> std::pair<size_t, std::vector<size_t>>
FvsInstance::remove_nodes(F &&f) {
  auto permutation = _graph.remove_nodes(f);

  size_t removed = 0;

  auto neighbors_to_excluded_permuted = std::unordered_set<size_t>();

  // We assume permutation[idx] <= idx for all idx.
  for (size_t idx = 0; idx < _original_indices.size(); idx++) {
    if (permutation[idx] < _graph.size()) {
      _original_indices[permutation[idx]] = _original_indices[idx];
      _exclusion_markers[permutation[idx]] = _exclusion_markers[idx];

      if (_neighbors_to_excluded.contains(idx)) {
        neighbors_to_excluded_permuted.insert(permutation[idx]);
      }
    } else {
      removed++;
    }
  }
  _original_indices.resize(_graph.size());
  _exclusion_markers.resize(_graph.size());
  _neighbors_to_excluded = neighbors_to_excluded_permuted;

  return {removed, permutation};
}
} // namespace datastructures