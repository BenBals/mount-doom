#pragma once

#include "base.h"

namespace algorithms {
class BruteforcingSolver : public Solver {
public:
  ~BruteforcingSolver() override = default;
  std::optional<PartialSolution> solve(FvsInstance &&instance) override;

public:
};
}