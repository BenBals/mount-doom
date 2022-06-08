#pragma once

#include "src/datastructures.h"

#include <optional>
#include <vector>

namespace algorithms {
using namespace datastructures;

class Solver {
public:
  virtual ~Solver() = default;

  virtual std::optional<PartialSolution> solve(FvsInstance &&instance) = 0;
};

/**
 * A Solver, that needs exactly one fallback solver, should be a Decorating Solver.
 */
class DecoratingSolver : public Solver {
  Solver *_inner = nullptr;

protected:
  std::optional<PartialSolution> solve_with_inner(FvsInstance &&instance);

public:
  DecoratingSolver() = default;
  explicit DecoratingSolver(Solver *inner);
  ~DecoratingSolver() override = default;

  Solver *inner() const;
  void inner(Solver *inner);
};

template <class F>
concept node_key_function = helpers::keying_function<F, size_t, const FvsInstance &>;

template <class F>
using node_key_function_value =
    std::invoke_result_t<F, size_t, const datastructures::FvsInstance &>;

template <class F>
requires node_key_function<F>
class NodeKeyed {
  F _key_function;

protected:
  node_key_function_value<F> get_node_key(size_t index, const FvsInstance &instance);

public:
  ~NodeKeyed() = default;

  explicit NodeKeyed(F &&key_function);
};

size_t degree_for_node_key(size_t index, const FvsInstance &instance);
size_t degree_for_node_only_directed_key(size_t index, const FvsInstance &instance);
} // namespace algorithms

#include "base_impl.h"