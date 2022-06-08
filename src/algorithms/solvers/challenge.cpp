#include "challenge.h"
namespace algorithms {
BranchingChallengeSolver::BranchingChallengeSolver()
    : _phase_3(std::make_unique<SccSplittingSolver>()),
      _phase_2(std::make_unique<TrivialInstanceSolver>(_phase_3.get())),
      _phase_1(std::make_unique<ReducingSolver>(_phase_2.get())) {

  auto strategy = std::make_unique<ChainingBranchingStrategy>();
  strategy->push_back(std::make_unique<MaxNeighborByDegreeBranchingStrategy>());
  strategy->push_back(std::make_unique<K2MaxDegreeBranchingStrategy>());
  strategy->push_back(std::make_unique<MaxByDegreeBranchingStrategy>());

  _phase_4 = std::make_unique<BranchingSolver>(std::move(strategy));

  _phase_4->inner(_phase_1.get());
  _phase_3->inner(_phase_4.get());
}
std::optional<std::vector<size_t>>
algorithms::BranchingChallengeSolver::solve(FvsInstance &&instance) {
  return _phase_1->solve(std::move(instance));
}
ArticulationPointBranchingChallengeSolver::ArticulationPointBranchingChallengeSolver()
    : _phase_3(std::make_unique<SccSplittingSolver>()),
      _phase_2(std::make_unique<TrivialInstanceSolver>(_phase_3.get())),
      _phase_1(std::make_unique<ReducingSolver>(_phase_2.get())) {
  auto strategy = std::make_unique<ChainingBranchingStrategy>();
  strategy->push_back(std::make_unique<NonK2MaxNonK2DegreeBranchingStrategy>());
  strategy->push_back(std::make_unique<MaxNeighborByDegreeBranchingStrategy>());
  strategy->push_back(std::make_unique<K2MaxDegreeBranchingStrategy>());
  strategy->push_back(std::make_unique<MaxByDegreeBranchingStrategy>());
  _phase_5 = std::make_unique<BranchingSolver>(std::move(strategy));
  _phase_5->inner(_phase_1.get());

  _phase_4 = std::make_unique<ArticulationPointSolver>();
  _phase_4->applicable_solver(_phase_1.get());
  _phase_4->not_applicable_solver(_phase_5.get());

  _phase_3->inner(_phase_4.get());
}
std::optional<std::vector<size_t>>
ArticulationPointBranchingChallengeSolver::solve(FvsInstance &&instance) {
  return _phase_1->solve(std::move(instance));
}
ArticulationPointVertexCoverBranchingChallengeSolver::
    ArticulationPointVertexCoverBranchingChallengeSolver()
    : _phase_3(std::make_unique<SccSplittingSolver>()),
      _phase_2(std::make_unique<TrivialInstanceSolver>(_phase_3.get())),
      _phase_1(std::make_unique<ReducingSolver>(_phase_2.get())) {
  auto strategy = std::make_unique<ChainingBranchingStrategy>();
  strategy->push_back(std::make_unique<NonK2MaxNonK2DegreeBranchingStrategy>());
  strategy->push_back(std::make_unique<MaxNeighborByDegreeBranchingStrategy>());
  strategy->push_back(std::make_unique<K2MaxDegreeBranchingStrategy>());
  strategy->push_back(std::make_unique<MaxByDegreeBranchingStrategy>());
  _phase_6 = std::make_unique<BranchingSolver>(std::move(strategy));
  _phase_6->inner(_phase_1.get());

  _phase_5 = std::make_unique<VertexCoverSolver>(_phase_6.get());

  _phase_4 = std::make_unique<ArticulationPointSolver>();
  _phase_4->applicable_solver(_phase_1.get());
  _phase_4->not_applicable_solver(_phase_5.get());

  _phase_3->inner(_phase_4.get());
}
std::optional<std::vector<size_t>>
ArticulationPointVertexCoverBranchingChallengeSolver::solve(FvsInstance &&instance) {
  return _phase_1->solve(std::move(instance));
}
CycleCliqueVertexCoverChallengeSolver::CycleCliqueVertexCoverChallengeSolver()
    : _phase_2(std::make_unique<CycleCliqueVertexCoverSolver>()),
      _phase_1(std::make_unique<ReducingSolver>(_phase_2.get())) {}
std::optional<std::vector<size_t>>
CycleCliqueVertexCoverChallengeSolver::solve(FvsInstance &&instance) {
  return _phase_1->solve(std::move(instance));
}
} // namespace algorithms
