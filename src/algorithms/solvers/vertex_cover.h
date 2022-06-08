#pragma once
#include "base.h"
#include "greedy.h"

namespace algorithms {

class VertexCoverSolver : public DecoratingSolver {
public:
  VertexCoverSolver() = default;
  explicit VertexCoverSolver(Solver *inner);
  ~VertexCoverSolver() override = default;
  std::optional<std::vector<size_t>> solve(FvsInstance &&instance) override;
};

/**
 * This solver solves the VC problem on the undirected edges. If this solves the DFVS this solution
 * is returned. Otherwise an upper bound is computed.
 */
class UnderlyingVertexCoverSolver : public DecoratingSolver {
public:
  UnderlyingVertexCoverSolver() = default;
  explicit UnderlyingVertexCoverSolver(Solver *inner);
  ~UnderlyingVertexCoverSolver() override = default;

  std::optional<std::vector<size_t>> solve(FvsInstance &&instance) override;
};

/**
 * This solver solves a DFVS instance by solving vertex cover and adding cliques to the remaining
 * cycles over and over again, until the vertex cover solves the DFVS.
 */
class CycleCliqueVertexCoverSolver : public Solver {
public:
  CycleCliqueVertexCoverSolver() = default;
  ~CycleCliqueVertexCoverSolver() override = default;
  std::optional<std::vector<size_t>> solve(FvsInstance &&instance) override;
};
} // namespace algorithms