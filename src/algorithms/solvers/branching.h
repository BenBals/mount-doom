#pragma once

#include "datastructures.h"
#include "helpers.h"

#include "base.h"

namespace algorithms {
/**
 * Specifies a strategy on how to choose the next branching vertex
 */
class BranchingStrategy {
public:
  virtual ~BranchingStrategy() = default;
  /**
   * @param instance
   * @return If this branching strategy is applicable, it returns a vector of instances, such that
   * the best instance has an optimal solution equal to an optimal solution of `instance`. If it is
   * not applicable it returns an empty optional.
   */
  virtual std::optional<std::vector<FvsInstance>> branching_instances(FvsInstance &&instance) = 0;
};

class ChainingBranchingStrategy : public BranchingStrategy {
  std::vector<std::unique_ptr<BranchingStrategy>> _inner_strategies;

public:
  ~ChainingBranchingStrategy() override = default;

  /**
   * Strategies are tried in order of insertion.
   * @param strategy
   */
  void push_back(std::unique_ptr<BranchingStrategy> &&strategy);

  std::optional<std::vector<FvsInstance>> branching_instances(FvsInstance &&instance) override;
};

template <class F>
  requires node_key_function<F>
class MaxByKeyBranchingStrategy : NodeKeyed<F>, public BranchingStrategy {
public:
  ~MaxByKeyBranchingStrategy() override = default;

  explicit MaxByKeyBranchingStrategy(F &&key_function);
  std::optional<std::vector<FvsInstance>> branching_instances(FvsInstance &&instance) override;
};

template <class F>
  requires node_key_function<F>
class K2BranchingStrategy : NodeKeyed<F>, public BranchingStrategy {
public:
  ~K2BranchingStrategy() override = default;

  explicit K2BranchingStrategy(F &&key_function);
  std::optional<std::vector<FvsInstance>> branching_instances(FvsInstance &&instance) override;
};

template <class F>
  requires node_key_function<F>
class MaxNeighborByKeyBranchingStrategy : NodeKeyed<F>, public BranchingStrategy {
public:
  ~MaxNeighborByKeyBranchingStrategy() override = default;

  explicit MaxNeighborByKeyBranchingStrategy(F &&key_function);
  std::optional<std::vector<FvsInstance>> branching_instances(FvsInstance &&instance) override;
};

template <class F>
  requires node_key_function<F>
class NonK2BranchingStrategy : NodeKeyed<F>, public BranchingStrategy {
public:
  ~NonK2BranchingStrategy() override = default;

  explicit NonK2BranchingStrategy(F &&key_function);
  std::optional<std::vector<FvsInstance>> branching_instances(FvsInstance &&instance) override;
};

template <class F>
  requires node_key_function<F>
class SmallCycleBranchingStrategy : NodeKeyed<F>, public BranchingStrategy {
private:
  Graph::CycleFindingData cycle_finding_data;

public:
  ~SmallCycleBranchingStrategy() override = default;

  explicit SmallCycleBranchingStrategy(F &&key_function);
  std::optional<std::vector<FvsInstance>> branching_instances(FvsInstance &&instance) override;
};

class MaxByDegreeBranchingStrategy
    : public MaxByKeyBranchingStrategy<decltype(&degree_for_node_key)> {
public:
  MaxByDegreeBranchingStrategy();
  ~MaxByDegreeBranchingStrategy() override = default;
};

class K2MaxDegreeBranchingStrategy : public K2BranchingStrategy<decltype(&degree_for_node_key)> {
public:
  K2MaxDegreeBranchingStrategy();
  ~K2MaxDegreeBranchingStrategy() override = default;
};

class NonK2MaxNonK2DegreeBranchingStrategy
    : public NonK2BranchingStrategy<decltype(&degree_for_node_only_directed_key)> {
public:
  NonK2MaxNonK2DegreeBranchingStrategy();
  ~NonK2MaxNonK2DegreeBranchingStrategy() override = default;
};

class MaxNeighborByDegreeBranchingStrategy
    : public MaxNeighborByKeyBranchingStrategy<decltype(&degree_for_node_key)> {
public:
  MaxNeighborByDegreeBranchingStrategy();
  ~MaxNeighborByDegreeBranchingStrategy() override = default;
};

class MaxDegreeInSmallCycleBranchingStrategy
    : public SmallCycleBranchingStrategy<decltype(&degree_for_node_key)> {
public:
  MaxDegreeInSmallCycleBranchingStrategy();
  ~MaxDegreeInSmallCycleBranchingStrategy() override = default;
};

class BranchingSolver : public DecoratingSolver {
  std::unique_ptr<BranchingStrategy> _strategy;

public:
  /**
   * @param strategy The BranchingStrategy to use. This must always produce at least one branching
   * option.
   */
  explicit BranchingSolver(std::unique_ptr<BranchingStrategy> &&strategy);
  BranchingSolver(std::unique_ptr<BranchingStrategy> &&strategy, Solver *inner);
  ~BranchingSolver() override = default;

  std::optional<PartialSolution> solve(FvsInstance &&instance) override;
};
} // namespace algorithms

#include "branching_impl.h"
