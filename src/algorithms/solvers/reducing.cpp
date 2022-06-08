#include "reducing.h"

namespace algorithms {
ReducingSolver::ReducingSolver() : DecoratingSolver() {}
ReducingSolver::ReducingSolver(Solver *inner) : DecoratingSolver(inner) {}
std::optional<std::vector<size_t>> ReducingSolver::solve(FvsInstance &&instance) {
  instance.reductions_exhaustively();

  return solve_with_inner(std::move(instance));
}

} // namespace algorithms::solvers
