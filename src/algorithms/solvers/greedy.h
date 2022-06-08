#pragma once

#include "base.h"

namespace algorithms {
/**
 * This solver first takes the nodes greedily into the dfvs (by maximal key according to the keying
 * function) until the remaining graph is acyclic. Then the nodes are checked in reverse order if
 * the computed set is a dfvs without this node. If it is, this node is remvoed from the dfvs.
 */
template <class F>
  requires node_key_function<F>
class MaxByKeyHeuristicSolver : NodeKeyed<F>, public Solver {
  size_t _reduce_after_taking_n = SIZE_MAX;
  Graph::CycleFindingData cycle_finding_data;

  std::optional<std::pair<size_t, std::vector<size_t>>> find_next_cycle(size_t start_node,
                                                                        const Graph &graph);

public:
  explicit MaxByKeyHeuristicSolver(F &&keying_function);
  MaxByKeyHeuristicSolver(F &&keying_function, size_t reduce_after_taking_n);
  ~MaxByKeyHeuristicSolver() override = default;

  std::optional<PartialSolution> solve(FvsInstance &&instance) override;
  PartialSolution greedily_find_initial_solution(FvsInstance &&instance);
  PartialSolution make_solution_nodes_minimal(const FvsInstance &instance,
                                              const std::vector<size_t>& unfiltered_solution);
  /**
   * This method uses a subroutine from Chen's algorithm, i.e. it implements the compression algo
   * but only 2 vertices are excluded from the dFVS and one is included
   * tl;dr: If you take two nodes v,u out of the solution set, all cycles must use these two nodes.
   * So if you hit all cycles (i.e. paths v~>v, u~>u) this is fine
   * @param unfiltered_solution Existent solution to the dFVS. Should be inclusion-minimal for reduced runtime. Must not have self-loops
   * @return Vector of node indices that form a dFVS. The solution is minimal regarding the 2-1-compression
   */
  PartialSolution iterative_small_compression(const FvsInstance &instance,
                                              const std::vector<size_t>& unfiltered_solution);
};

class MaxByDegreeHeuristicSolver : public MaxByKeyHeuristicSolver<decltype(&degree_for_node_key)> {
public:
  MaxByDegreeHeuristicSolver();
  explicit MaxByDegreeHeuristicSolver(size_t reduce_after_taking_n);
};
} // namespace algorithms

#include "greedy_impl.h"
