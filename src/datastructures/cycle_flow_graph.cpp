#include <numeric>
#include <queue>

#include "cycle_flow_graph.h"

namespace datastructures {

CycleFlowGraph::CycleFlowGraph(const Graph &graph)
    : _graph(graph.size() * 2), _parent(graph.size() * 2), _last_visited(graph.size() * 2) {
  for (size_t idx = 0; idx < graph.size(); idx++) {
    for (size_t out_idx : graph[idx].out_edges()) {
      _graph.add_arc_unsafe(2 * idx + 1, 2 * out_idx);
    }
    _graph.add_arc_unsafe(2 * idx, 2 * idx + 1);
  }
}
bool CycleFlowGraph::bfs(size_t from_idx, size_t to_idx, size_t iteration) {
  std::queue<size_t> bfs_q;
  _last_visited[from_idx] = iteration;
  bfs_q.push(from_idx);

  while (!bfs_q.empty() && _last_visited[to_idx] != iteration) {
    size_t cur_idx = bfs_q.front();
    bfs_q.pop();

    for (size_t next_idx : _graph[cur_idx].out_edges()) {
      if (_last_visited[next_idx] != iteration) {
        _parent[next_idx] = cur_idx;
        _last_visited[next_idx] = iteration;
        bfs_q.push(next_idx);
      }
    }
  }

  return _last_visited[to_idx] == iteration;
}
void CycleFlowGraph::build_residual_graph(size_t from_idx, size_t to_idx) {
  size_t parent_idx = _parent[to_idx];
  size_t cur_idx = to_idx;

  _graph.swap_arc_unsafe(parent_idx, cur_idx);
  _swapped_arcs.push({cur_idx, parent_idx});

  while (parent_idx != from_idx) {
    cur_idx = parent_idx;
    parent_idx = _parent[parent_idx];
    _graph.swap_arc_unsafe(parent_idx, cur_idx);
    _swapped_arcs.push({cur_idx, parent_idx});
  }
}
void CycleFlowGraph::reset_residual_graph() {
  while (!_swapped_arcs.empty()) {
    auto edge = _swapped_arcs.top();
    _swapped_arcs.pop();
    _graph.swap_arc_unsafe(edge.first, edge.second);
  }
}
size_t CycleFlowGraph::get_max_flow(size_t source_idx, size_t sink_idx, size_t max_flow) {
  bool path_found = true;
  size_t num_found_cycles = 0;
  while (path_found && num_found_cycles < max_flow) {
    path_found = this->bfs(source_idx, sink_idx, ++_current_iteration);
    if (path_found) {
      this->build_residual_graph(source_idx, sink_idx);
      num_found_cycles++;
    }
  }
  this->reset_residual_graph();
  return num_found_cycles;
}
} // namespace datastructures