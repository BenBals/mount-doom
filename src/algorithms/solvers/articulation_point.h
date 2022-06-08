#pragma once

#include "base.h"

namespace algorithms {
/**
 * This solver tries to make progress using splitting instances at articulation points.
 * The rules and proofs can be found in the notion at
 * https://www.notion.so/Branching-on-Articulation-Points-8ed6b939e8c3432c84c24564a676daff
 *
 */
class ArticulationPointSolver : public Solver {

  Solver *_applicable_solver = nullptr, *_not_applicable_solver = nullptr;

protected:
  std::optional<PartialSolution> solve_applicable(FvsInstance &&instance);
  std::optional<PartialSolution> solve_not_applicable(FvsInstance &&instance);

public:
  ArticulationPointSolver() = default;
  /**
   * @param applicableSolver Use this solver, when a cut vertex was found i.e. this rule is
   * applicable
   * @param notApplicableSolver Use this solver, when no cut vertex was found i.e. this rule is not
   * applicable
   */
  ArticulationPointSolver(Solver *applicableSolver, Solver *notApplicableSolver);

  Solver *applicable_solver() const;
  void applicable_solver(Solver *inner);

  Solver *not_applicable_solver() const;
  void not_applicable_solver(Solver *inner);

  std::optional<PartialSolution> solve(FvsInstance &&instance) override;
};
} // namespace algorithms