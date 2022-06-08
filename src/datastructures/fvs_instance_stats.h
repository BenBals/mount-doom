#pragma once

#include <array>

#include "fvs_instance.h"
#include <array>
namespace datastructures {
struct FvsInstanceStats {
  double num_nodes = 0;
  double num_arcs = 0;
  double partial_fvs_size = 0;
  std::array<double, 5> out_degree_quartiles{};

  FvsInstanceStats() = default;
  explicit FvsInstanceStats(const FvsInstance &instance);

  void operator+=(const FvsInstanceStats &other);
  void operator*=(double factor);
  std::string to_string();
};

FvsInstanceStats meanGraphStats(const std::vector<FvsInstanceStats> &stats);
FvsInstanceStats meanGraphStatsForInstances(const std::vector<FvsInstance> &instances);
} // namespace datastructures
