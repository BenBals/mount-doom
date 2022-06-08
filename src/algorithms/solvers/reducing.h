#pragma once

#include "base.h"

namespace algorithms {
class ReducingSolver : public DecoratingSolver {
public:
  ReducingSolver();
  explicit ReducingSolver(Solver *inner);
  ~ReducingSolver() override = default;

  std::optional<std::vector<size_t>> solve(FvsInstance &&instance) override;
};
} // namespace algorithms::solvers
