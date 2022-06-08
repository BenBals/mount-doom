#pragma once

#include <memory>
#include <vector>

namespace datastructures {

class PartialSolutionFinalizer {
public:
  virtual ~PartialSolutionFinalizer() = default;
  /**
   * @return The number of nodes that finalize_solution will add to the solution
   */
  virtual size_t number_of_eventually_added_nodes() const = 0;
  /**
   * @param solution Undoes a reduction on the solution
   */
  virtual void finalize_solution(std::vector<size_t> &solution) const = 0;
};

class PartialSolutionDegreeTwoFinalizer : public PartialSolutionFinalizer {
  size_t _inner, _outer1, _outer2, _in_solution;

public:
  PartialSolutionDegreeTwoFinalizer(size_t inner, size_t outer1, size_t outer2, size_t in_solution);
  ~PartialSolutionDegreeTwoFinalizer() override = default;

  size_t number_of_eventually_added_nodes() const override;
  void finalize_solution(std::vector<size_t> &solution) const override;
};

class PartialSolutionFunnelFoldFinalizer : public PartialSolutionFinalizer {
  std::vector<size_t> _clique;
  size_t _to_take_with_full_clique, _to_take_without_full_clique;

public:
  PartialSolutionFunnelFoldFinalizer(std::vector<size_t> &&clique, size_t to_take_with_full_clique,
                                     size_t to_take_without_full_clique);
  ~PartialSolutionFunnelFoldFinalizer() override = default;

  size_t number_of_eventually_added_nodes() const override;
  void finalize_solution(std::vector<size_t> &solution) const override;
};

class PartialSolution {
  std::vector<size_t> _chosen_nodes;
  size_t _will_additionally_be_chosen = 0;

  std::vector<std::shared_ptr<PartialSolutionFinalizer>> _finalizers;

public:
  PartialSolution() = default;
  /**
   * Adds the node with the original index to the solution
   */
  void add_node(size_t original_index);

  /**
   * First gives the three vertices which were unified by a degree 2 reduction. As forth argument
   * the original index of the unified node is taken.
   */
  void add_degree_2_unification(size_t inner, size_t outer1, size_t outer2, size_t in_graph);
  /**
   * Remembers, that a funnel fold occured. If all of clique are chosen in the end, then
   * to_take_with_full_clique is added to the solution. Otherwise to_take_without_full_clique is
   * added.
   */
  void add_funnel_expansion(std::vector<size_t> &&clique, size_t to_take_with_full_clique,
                            size_t to_take_without_full_clique);

  /**
   * Gives the number of nodes, that in the original graph will be chosen by this partial solution.
   */
  size_t number_of_final_nodes() const;

  /**
   * @return The vertices of this solution in the original graph. This may only be called once an
   * instance is solved completely.
   */
  const std::vector<size_t> &solution_nodes();

  /**
   * @return The original indices of the vertices chosen to be in this solution. Beware finalizers
   * are not applied, so use with caution.
   */
  const std::vector<size_t> &chosen_nodes_without_finalization();

  /**
   * Merges the other Partial solution into this one. The order is important and the other solution
   * should solve a subgraph of the original instance to which this refers.
   */
  void merge(PartialSolution &&other);

  void clear();
};
} // namespace datastructures