#include "base.h"
#include "helpers.h"

namespace algorithms {
DecoratingSolver::DecoratingSolver(Solver *inner) : _inner(inner) {}
Solver *DecoratingSolver::inner() const { return _inner; }
void DecoratingSolver::inner(Solver *inner) { _inner = inner; }
std::optional<std::vector<size_t>> DecoratingSolver::solve_with_inner(FvsInstance &&instance) {
  helpers::assert_and_log(_inner != nullptr, "Inner solver was not set");
  return _inner->solve(std::move(instance));
}

size_t degree_for_node_key(size_t index, const FvsInstance &instance) {
  return instance._graph[index].all_edges().size();
}
size_t degree_for_node_only_directed_key(size_t index, const FvsInstance &instance) {
  return instance._graph[index].all_edges_not_undirected().size();
}
} // namespace algorithms
