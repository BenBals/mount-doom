#include "branching.h"

#include <utility>

namespace algorithms {
void ChainingBranchingStrategy::push_back(std::unique_ptr<BranchingStrategy> &&strategy) {
  _inner_strategies.emplace_back(std::move(strategy));
}

std::optional<std::vector<FvsInstance>>
ChainingBranchingStrategy::branching_instances(FvsInstance &&instance) {
  const auto initial_instance = instance;
  for (auto &inner : _inner_strategies) {
    auto result = inner->branching_instances(std::move(instance));
    if (result)
      return result;

    // restore instance
    instance = initial_instance;
  }
  return {};
}

MaxByDegreeBranchingStrategy::MaxByDegreeBranchingStrategy()
    : MaxByKeyBranchingStrategy(&degree_for_node_key) {}

K2MaxDegreeBranchingStrategy::K2MaxDegreeBranchingStrategy()
    : K2BranchingStrategy(&degree_for_node_key) {}

NonK2MaxNonK2DegreeBranchingStrategy::NonK2MaxNonK2DegreeBranchingStrategy()
    : NonK2BranchingStrategy(&degree_for_node_only_directed_key) {}

MaxNeighborByDegreeBranchingStrategy::MaxNeighborByDegreeBranchingStrategy()
    : MaxNeighborByKeyBranchingStrategy(&degree_for_node_key) {}

MaxDegreeInSmallCycleBranchingStrategy::MaxDegreeInSmallCycleBranchingStrategy()
    : SmallCycleBranchingStrategy(&degree_for_node_key) {}

BranchingSolver::BranchingSolver(std::unique_ptr<BranchingStrategy> &&strategy)
    : DecoratingSolver(), _strategy(std::move(strategy)) {}
BranchingSolver::BranchingSolver(std::unique_ptr<BranchingStrategy> &&strategy, Solver *inner)
    : DecoratingSolver(inner), _strategy(std::move(strategy)) {}
std::optional<PartialSolution> BranchingSolver::solve(FvsInstance &&instance) {
  spdlog::debug("\t\t\tBranching n={}, pss={}", instance._graph.size(),
                instance.partial_solution_size());

  auto bound = instance._upper_bound;
  auto sub_instances_opt = _strategy->branching_instances(std::move(instance));
  helpers::assert_and_log(sub_instances_opt.has_value(),
                          "The branching strategy must always be applicable");

  auto sub_instances = sub_instances_opt.value();
  helpers::assert_and_log(!sub_instances.empty(), "The branching strategy provided no branch");

  std::optional<PartialSolution> best;
  for (auto &sub_instance : sub_instances) {
    sub_instance._upper_bound = bound;
    auto result = solve_with_inner(std::move(sub_instance));
    if (result) {
      spdlog::debug("\t\t\tImproved bound from {} to {}", bound, result.value().number_of_final_nodes());
      bound = result.value().number_of_final_nodes();
      best = result;
    }
  }
  return best;
}
} // namespace algorithms