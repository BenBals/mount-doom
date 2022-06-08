#include "scc_splitting.h"
#include "helpers.h"

namespace algorithms {
SccSplittingSolver::SccSplittingSolver(Solver *inner) : DecoratingSolver(inner) {}
std::optional<std::vector<size_t>> SccSplittingSolver::solve(FvsInstance &&input) {

  auto bound = input._upper_bound;
  auto [scc_instances, solution] = split_fvs_instance_by_sccs(std::move(input));
  std::sort(scc_instances.begin(), scc_instances.end(),
            [](const auto &instance1, const auto &instance2) {
              return instance1._graph.size() < instance2._graph.size();
            });
  if (solution.size() >= bound)
    return {};

  bound -= solution.size();
  for (auto &scc_instance : scc_instances) {
    scc_instance._upper_bound = bound;
    auto sub_solution_opt = solve_with_inner(std::move(scc_instance));
    if (!sub_solution_opt)
      return {};

    auto sub_solution = sub_solution_opt.value();
    if (sub_solution.size() >= bound)
      return {};
    bound -= sub_solution.size();
    std::copy(sub_solution.begin(), sub_solution.end(), std::back_inserter(solution));
  }
  return {solution};
}

} // namespace algorithms
