#pragma once

namespace datastructures {
template <class F>
requires std::predicate<F, Graph::Node &> std::vector<size_t> Graph::remove_nodes(F &&f) {
  size_t removed = 0;
  std::vector<size_t> new_indices(size(), size());
  std::iota(new_indices.begin(), new_indices.end(), 0);

  for (size_t i = 0; i < size(); i++) {
    const auto &node = (*this)[i];
    if (f(node)) {
      removed++;
      new_indices[i] = size();
    } else {
      if (removed > 0)
        _nodes[i - removed] =
            std::move(_nodes[i]); // moving from and to the same object is not allowed
      new_indices[i] = i - removed;
    }
  }

  if (removed == 0) {
    return new_indices;
  }
  _nodes.resize(size() - removed);

  // fixup the remaining edges
  for (size_t idx = 0; idx < size(); idx++) {
    auto &node = (*this)[idx];
    auto fixup = [&](auto &vec) {
      std::erase_if(vec, [&](const auto other) { return new_indices[other] >= _nodes.size(); });
      std::transform(vec.begin(), vec.end(), vec.begin(),
                     [&](auto &other) { return new_indices[other]; });
    };
    fixup(node._in);
    fixup(node._out);
    fixup(node._undirected);
    node._idx = idx;
  }

  return new_indices;
}

template <class F>
requires std::predicate<F, Graph::Node &, Graph::EdgeDirection>
void Graph::remove_neighbors_of_node_unsafe(size_t idx, F &&f) {
  auto &node = _nodes[idx];
  for (auto pair : {std::pair(&node._undirected, EdgeDirection::UNDIRECTED),
                    std::pair(&node._in, EdgeDirection::INCOMING),
                    std::pair(&node._out, EdgeDirection::OUTGOING)}) {
    std::vector<size_t> *neighbor_vec_ptr;
    Graph::EdgeDirection edge_direction;
    tie(neighbor_vec_ptr, edge_direction) = pair;
    std::erase_if(*neighbor_vec_ptr, [edge_direction, &f, this](size_t neighbor_idx) {
      return f(_nodes[neighbor_idx], edge_direction);
    });
  }
}

template <class F>
std::pair<size_t, std::vector<size_t>>
Graph::strongly_connected_components_inner(F &&edges_of_node) const {
  // Tarjan's algorithm for scc, the code is from kactl
  // (https://github.com/kth-competitive-programming/kactl/blob/main/content/graph/SCC.h)
  // Variable names are left as in kactl due to the lack of the copyist's understanding of this
  // algorithm
  std::vector<size_t> component(size(), size()), val(size(), 0), z;
  size_t time = 0, cur_component = 0;

  auto dfs = [&](size_t cur, auto &&rec) -> size_t {
    time++;
    val[cur] = time;
    size_t low = time;
    z.push_back(cur);

    auto &node = _nodes[cur];
    for (const auto neighbor : edges_of_node(node)) {
      if (component[neighbor] == size()) {
        low = std::min(low, val[neighbor] > 0 ? val[neighbor] : rec(neighbor, rec));
      }
    }

    if (low == val[cur]) {
      size_t x;
      do {
        x = z.back();
        z.pop_back();
        component[x] = cur_component;
      } while (x != cur);
      cur_component++;
    }
    val[cur] = low;
    return low;
  };

  for (size_t idx = 0; idx < size(); idx++) {
    if (component[idx] == size()) {
      dfs(idx, dfs);
    }
  }
  return {cur_component, component};
}
} // namespace datastructures