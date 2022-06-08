#include "trivial_instance.h"

namespace algorithms {
TrivialInstanceSolver::TrivialInstanceSolver(Solver *inner) : DecoratingSolver(inner) {}
std::optional<PartialSolution> TrivialInstanceSolver::solve(FvsInstance &&instance) {
  auto lower_bound = instance.lower_bound();
  if (instance.partial_solution_size() + lower_bound >= instance._upper_bound) {
    return {};
  }
  if (lower_bound == 0) {
    // lower bound is 0 iff the instance is solved
    return {std::move(instance._partial_solution)};
  }
  return solve_with_inner(std::move(instance));
}

} // namespace algorithms
