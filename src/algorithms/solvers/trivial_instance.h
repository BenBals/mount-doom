#pragma once

#include "base.h"

namespace algorithms {
/**
 * This solver takes care of reducing trivial no instances (solutions size >= bound) and trivial yes instance (empty or acyclic)
 */
class TrivialInstanceSolver : public DecoratingSolver {
public:
  TrivialInstanceSolver() = default;
  explicit TrivialInstanceSolver(Solver *inner);
  ~TrivialInstanceSolver() override = default;

  std::optional<PartialSolution> solve(FvsInstance &&instance) override;
};
}