#include "brute_forcing.h"

#include <bit>

std::optional<std::vector<size_t>> algorithms::BruteforcingSolver::solve(FvsInstance &&input) {
  std::optional<std::vector<size_t>> best = {};
  auto instance = input;

  helpers::assert_and_log(instance._graph.size() < 64, "graph is too big to brute force");
  for (size_t characteristic = 0; characteristic < 1ul << instance._graph.size();
       characteristic++) {

    if (best.has_value() &&
        static_cast<size_t>(std::popcount(characteristic)) >= best.value().size())
      continue;
    std::vector<size_t> solution;
    for (size_t idx = 0; idx < instance._graph.size(); idx++) {
      if (characteristic & (1 << idx)) {
        solution.push_back(instance._original_indices[idx]);
      }
    }
    instance._partial_solution = solution;

    if (instance.is_solved()) {
      best = solution;
    }
  }
  helpers::assert_and_log(best.has_value(), "brute force did not find a solution");

  std::copy(input._partial_solution.begin(), input._partial_solution.end(),
            std::back_inserter(best.value()));

  return best;
}
