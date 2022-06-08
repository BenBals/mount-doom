#pragma once

#include "base.h"

namespace algorithms {
template <class F>
requires node_key_function<F> NodeKeyed<F>::NodeKeyed(F &&key_function)
    : _key_function(std::move(key_function)) {}

template <class F>
requires node_key_function<F> node_key_function_value<F>
NodeKeyed<F>::get_node_key(size_t index, const FvsInstance &instance) {
  return _key_function(index, instance);
}
} // namespace algorithms