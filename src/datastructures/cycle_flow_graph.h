#pragma once

#include "graph.h"
#include <vector>

namespace datastructures {

struct CycleFlowGraph {
  Graph _graph;
  std::vector<size_t> _parent;
  std::vector<size_t> _last_visited;
  size_t _current_iteration = 0;
  std::stack<std::pair<size_t, size_t>, std::vector<std::pair<size_t, size_t>>> _swapped_arcs;

  explicit CycleFlowGraph(const Graph &graph);

public:
  size_t get_max_flow(size_t source_idx, size_t sink_idx, size_t max_flow);

private:
  bool bfs(size_t from_idx, size_t to_idx, size_t iteration);
  void build_residual_graph(size_t from_idx, size_t to_idx);
  void reset_residual_graph();
};

}