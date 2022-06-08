#include "brute_forcing.h"

#include <bit>

namespace algorithms {
using namespace datastructures;

std::optional<PartialSolution> BruteforcingSolver::solve(FvsInstance &&input) {
  std::optional<PartialSolution> best = {};

  helpers::assert_and_log(input._graph.size() < 64, "graph is too big to brute force");
  for (size_t characteristic = 0; characteristic < 1ul << input._graph.size(); characteristic++) {

    if (best.has_value() && input.partial_solution_size() +
                                    static_cast<size_t>(std::popcount(characteristic)) >=
                                best->number_of_final_nodes())
      continue;

    auto instance_copy = input;
    for (size_t idx = 0; idx < instance_copy._graph.size(); idx++) {
      if (characteristic & (1 << idx)) {
        instance_copy.take_node_into_partial_solution(idx);
      }
    }

    if (instance_copy._graph.is_acyclic()) {
      best = instance_copy._partial_solution;
    }
  }
  helpers::assert_and_log(best.has_value(), "brute force did not find a solution");

  return best;
}
} // namespace algorithms