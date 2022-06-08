#pragma once

#include "base.h"

namespace algorithms {
class SccSplittingSolver : public DecoratingSolver {
public:
  SccSplittingSolver() = default;
  SccSplittingSolver(Solver *inner);
  ~SccSplittingSolver() override = default;

  std::optional<std::vector<size_t>> solve(FvsInstance &&instance) override;
};
} // namespace algorithms