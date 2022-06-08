#pragma once

#include "base.h"

namespace algorithms {
class BruteforcingSolver : public Solver {
public:
  ~BruteforcingSolver() override = default;
  std::optional<std::vector<size_t>> solve(FvsInstance &&instance) override;

public:
};
}