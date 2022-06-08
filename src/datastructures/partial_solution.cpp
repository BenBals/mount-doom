#include "partial_solution.h"
#include <ranges>

namespace datastructures {
PartialSolutionDegreeTwoFinalizer::PartialSolutionDegreeTwoFinalizer(size_t inner, size_t outer1,
                                                                     size_t outer2,
                                                                     size_t in_solution)
    : _inner(inner), _outer1(outer1), _outer2(outer2), _in_solution(in_solution) {}
size_t PartialSolutionDegreeTwoFinalizer::number_of_eventually_added_nodes() const { return 1; }
void PartialSolutionDegreeTwoFinalizer::finalize_solution(std::vector<size_t> &solution) const {
  auto original_solution_size = solution.size();

  std::erase(solution, _in_solution);
  if (solution.size() < original_solution_size) {
    // in solution was deleted so we have to add both outer
    solution.push_back(_outer1);
    solution.push_back(_outer2);
  } else {
    // otherwise we only need to add the middle one
    solution.push_back(_inner);
  }
}
void PartialSolution::add_node(size_t node) { _chosen_nodes.push_back(node); }
void PartialSolution::add_degree_2_unification(size_t inner, size_t outer1, size_t outer2,
                                               size_t in_graph) {
  _finalizers.emplace_back(
      std::make_shared<PartialSolutionDegreeTwoFinalizer>(inner, outer1, outer2, in_graph));
  _will_additionally_be_chosen += _finalizers.back()->number_of_eventually_added_nodes();
}
void PartialSolution::add_funnel_expansion(std::vector<size_t> &&clique,
                                           size_t to_take_with_full_clique,
                                           size_t to_take_without_full_clique) {
  _finalizers.emplace_back(std::make_shared<PartialSolutionFunnelFoldFinalizer>(
      std::move(clique), to_take_with_full_clique, to_take_without_full_clique));
  _will_additionally_be_chosen += _finalizers.back()->number_of_eventually_added_nodes();
}
size_t PartialSolution::number_of_final_nodes() const {
  return _chosen_nodes.size() + _will_additionally_be_chosen;
}
const std::vector<size_t> &PartialSolution::solution_nodes() {
  while (!_finalizers.empty()) {
    auto &last_finalizer = _finalizers.back();

    _will_additionally_be_chosen -= last_finalizer->number_of_eventually_added_nodes();
    last_finalizer->finalize_solution(_chosen_nodes);

    _finalizers.pop_back();
  }
  return _chosen_nodes;
}
void PartialSolution::merge(PartialSolution &&other) {
  std::move(other._chosen_nodes.begin(), other._chosen_nodes.end(),
            std::back_inserter(_chosen_nodes));
  std::move(other._finalizers.begin(), other._finalizers.end(), std::back_inserter(_finalizers));
  _will_additionally_be_chosen += other._will_additionally_be_chosen;
}
void PartialSolution::clear() {
  _chosen_nodes.clear();
  _finalizers.clear();
  _will_additionally_be_chosen = 0;
}
const std::vector<size_t> &PartialSolution::chosen_nodes_without_finalization() {
  return _chosen_nodes;
}
PartialSolutionFunnelFoldFinalizer::PartialSolutionFunnelFoldFinalizer(
    std::vector<size_t> &&clique, size_t to_take_with_full_clique,
    size_t to_take_without_full_clique)
    : _clique(std::move(clique)), _to_take_with_full_clique(to_take_with_full_clique),
      _to_take_without_full_clique(to_take_without_full_clique) {
  std::sort(_clique.begin(), _clique.end());
}
size_t PartialSolutionFunnelFoldFinalizer::number_of_eventually_added_nodes() const { return 1; }
void PartialSolutionFunnelFoldFinalizer::finalize_solution(std::vector<size_t> &solution) const {
  size_t num_clique_nodes = 0;
  for (auto in_solution : solution) {
    if (std::binary_search(_clique.begin(), _clique.end(), in_solution))
      num_clique_nodes++;
  }

  if (num_clique_nodes == _clique.size()) {
    solution.push_back(_to_take_with_full_clique);
  } else {
    solution.push_back(_to_take_without_full_clique);
  }
}
} // namespace datastructures
