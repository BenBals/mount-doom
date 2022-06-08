#pragma once

#include "src/algorithms/solvers.h"
#include "vertex_cover.h"

namespace algorithms {
class BranchingChallengeSolver : public Solver {
  std::unique_ptr<BranchingSolver> _phase_4;
  std::unique_ptr<SccSplittingSolver> _phase_3;
  std::unique_ptr<TrivialInstanceSolver> _phase_2;
  std::unique_ptr<ReducingSolver> _phase_1;

public:
  BranchingChallengeSolver();
  ~BranchingChallengeSolver() override = default;

  std::optional<std::vector<size_t>> solve(FvsInstance &&instance) override;
};
class ArticulationPointBranchingChallengeSolver : public Solver {
  std::unique_ptr<BranchingSolver> _phase_5;
  std::unique_ptr<ArticulationPointSolver> _phase_4;
  std::unique_ptr<SccSplittingSolver> _phase_3;
  std::unique_ptr<TrivialInstanceSolver> _phase_2;
  std::unique_ptr<ReducingSolver> _phase_1;

public:
  ArticulationPointBranchingChallengeSolver();
  ~ArticulationPointBranchingChallengeSolver() override = default;

  std::optional<std::vector<size_t>> solve(FvsInstance &&instance) override;
};
class ArticulationPointVertexCoverBranchingChallengeSolver : public Solver {
  std::unique_ptr<BranchingSolver> _phase_6;
  std::unique_ptr<VertexCoverSolver> _phase_5;
  std::unique_ptr<ArticulationPointSolver> _phase_4;
  std::unique_ptr<SccSplittingSolver> _phase_3;
  std::unique_ptr<TrivialInstanceSolver> _phase_2;
  std::unique_ptr<ReducingSolver> _phase_1;

public:
  ArticulationPointVertexCoverBranchingChallengeSolver();
  ~ArticulationPointVertexCoverBranchingChallengeSolver() override = default;

  std::optional<std::vector<size_t>> solve(FvsInstance &&instance) override;
};
class CycleCliqueVertexCoverChallengeSolver : public Solver {
  std::unique_ptr<CycleCliqueVertexCoverSolver> _phase_2;
  std::unique_ptr<ReducingSolver> _phase_1;

public:
  CycleCliqueVertexCoverChallengeSolver();
  ~CycleCliqueVertexCoverChallengeSolver() override = default;

  std::optional<std::vector<size_t>> solve(FvsInstance &&instance) override;
};
} // namespace algorithms