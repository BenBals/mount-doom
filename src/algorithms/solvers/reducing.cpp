#include "reducing.h"

namespace algorithms {
ReducingSolver::ReducingSolver() : DecoratingSolver() {}
ReducingSolver::ReducingSolver(Solver *inner) : DecoratingSolver(inner) {}
std::optional<PartialSolution> ReducingSolver::solve(FvsInstance &&instance) {
  instance.reductions_exhaustively();

  return solve_with_inner(std::move(instance));
}

LightlyReducingSolver::LightlyReducingSolver(Solver *inner) : DecoratingSolver(inner) {}
std::optional<PartialSolution> LightlyReducingSolver::solve(FvsInstance &&instance) {
  instance.light_reductions_exhaustively();

  return solve_with_inner(std::move(instance));
}
} // namespace algorithms
