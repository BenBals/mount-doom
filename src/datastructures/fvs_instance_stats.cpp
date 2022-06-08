#include "fvs_instance_stats.h"

#include "fmt/core.h"
#include "spdlog/spdlog.h"

#include <string>
#include <vector>

namespace datastructures {
void FvsInstanceStats::operator+=(const FvsInstanceStats &other) {
  num_nodes += other.num_nodes;
  num_arcs += other.num_arcs;
  partial_fvs_size += other.partial_fvs_size;
  for (size_t i = 0; i < out_degree_quartiles.size(); i++) {
    out_degree_quartiles[i] += other.out_degree_quartiles[i];
  }
}
void FvsInstanceStats::operator*=(double factor) {
  num_nodes *= factor;
  num_arcs *= factor;
  partial_fvs_size *= factor;
  for (size_t i = 0; i < out_degree_quartiles.size(); i++) {
    out_degree_quartiles[i] *= factor;
  }
}
std::string FvsInstanceStats::to_string() {
  return fmt::format("=== Graph Stats\n"
                     "\tNum Nodes: {}\n"
                     "\tNum Arcs: {}\n"
                     "\tPartial FVS size: {}\n"
                     "\tOut Degree Quartiles: [{}]\n",
                     num_nodes, num_arcs, partial_fvs_size, fmt::join(out_degree_quartiles, ", "));
}
FvsInstanceStats::FvsInstanceStats(const FvsInstance &instance) {
  num_nodes = static_cast<double>(instance._graph.size());
  num_arcs = 0.0;
  partial_fvs_size = static_cast<double>(instance.partial_solution_size());

  std::vector<double> out_degrees;
  out_degrees.resize(instance._graph.size());

  for (size_t i = 0; i < instance._graph.size(); i++) {
    num_arcs += static_cast<double>(instance._graph[i].out_edges().size());
    out_degrees[i] = static_cast<double>(instance._graph[i].out_edges().size());
  }

  sort(out_degrees.begin(), out_degrees.end());
  if (out_degrees.empty())
    out_degrees.push_back(0);

  out_degree_quartiles = {
      out_degrees[0],
      out_degrees[1 * instance._graph.size() / 4],
      out_degrees[2 * instance._graph.size() / 4],
      out_degrees[3 * instance._graph.size() / 4],
      out_degrees[instance._graph.size() - 1],
  };
}

FvsInstanceStats meanGraphStats(const std::vector<FvsInstanceStats> &stats) {
  FvsInstanceStats agg;

  for (const auto &stat : stats) {
    agg += stat;
  }

  agg *= 1.0 / (static_cast<double>(stats.size()));

  return agg;
}
FvsInstanceStats meanGraphStatsForInstances(const std::vector<FvsInstance> &instances) {
  std::vector<FvsInstanceStats> stats(instances.size());

  for (size_t i = 0; i < stats.size(); i++) {
    stats[i] = FvsInstanceStats(instances[i]);
  }

  return meanGraphStats(stats);
}
} // namespace datastructures
