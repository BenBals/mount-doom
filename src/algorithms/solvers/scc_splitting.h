#pragma once

#include "base.h"

namespace algorithms {
class SccSplittingSolver : public DecoratingSolver {
public:
  SccSplittingSolver() = default;
  SccSplittingSolver(Solver *inner);
  ~SccSplittingSolver() override = default;

  std::optional<PartialSolution> solve(FvsInstance &&instance) override;
};
} // namespace algorithms