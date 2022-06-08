#pragma once

#include "base.h"

namespace algorithms {
class ReducingSolver : public DecoratingSolver {
public:
  ReducingSolver();
  explicit ReducingSolver(Solver *inner);
  ~ReducingSolver() override = default;

  std::optional<PartialSolution> solve(FvsInstance &&instance) override;
};
class LightlyReducingSolver : public DecoratingSolver {
public:
  LightlyReducingSolver() = default;
  explicit LightlyReducingSolver(Solver *inner);
  ~LightlyReducingSolver() override = default;

  std::optional<PartialSolution> solve(FvsInstance &&instance) override;
};
} // namespace algorithms
