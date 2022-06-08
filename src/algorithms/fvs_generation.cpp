#include "fvs_generation.h"
#include "helpers.h"

#include <random>

namespace algorithms {
FvsInstance generate_overlapping_cycles(size_t cycles, size_t cycle_size, size_t random_arcs) {
  helpers::assert_and_log(cycle_size > 2, "cycle size should be > 2");
  auto name =
      fmt::format("random overlapping cycles (num cycles = {}, size cycles = {}, extra arc = {})",
                  cycles, cycle_size, random_arcs);
  size_t num_nodes = cycles * (cycle_size - 1) + 1;
  Graph graph(num_nodes);

  for (size_t i = 0; i < num_nodes - 1; i++) {
    graph.add_arc(i, i + 1);
    if (i % (cycle_size - 1) == 0) {
      graph.add_arc(i + cycle_size - 1, i);
    }
  }

  // ensure the instance is not eaten up by reduction rules
  graph.add_arc(1, 0);
  graph.add_arc(num_nodes - 1, num_nodes - 2);

  std::mt19937 gen(42);

  while (random_arcs--) {
    size_t node1 = gen() % num_nodes, node2 = gen() % num_nodes;
    if (node1 == node2)
      continue;
    if (node1 > node2)
      std::swap(node1, node2);
    graph.add_arc(node1, node2);
  }

  FvsInstance instance(graph);
  instance.set_solution_size((cycles + 1) / 2 + 1);
  instance.permute();
  instance._name = std::move(name);
  std::iota(instance._original_indices.begin(), instance._original_indices.end(), 1);
  return instance;
}

FvsInstance generate_wide_cycle(size_t perimeter, size_t width) {
  helpers::assert_and_log(perimeter > 1, "perimeter should be > 1");
  helpers::assert_and_log(width > 0, "perimeter should be > 0");
  size_t num_nodes = perimeter * width;
  Graph graph(num_nodes);
  for (size_t i = 0; i < perimeter; i++) {
    for (size_t from = 0; from < width; from++) {
      for (size_t to = 0; to < width; to++) {
        graph.add_arc(i * width + from, ((i + 1) * width + to) % num_nodes);
      }
    }
  }

  FvsInstance instance(graph);
  instance.set_solution_size(width);
  instance.permute();
  instance._name = fmt::format("wide cycles (perimeter = {}, width = {})", perimeter, width);
  std::iota(instance._original_indices.begin(), instance._original_indices.end(), 1);
  return instance;
}

// Input:
// num_nodes must be even
FvsInstance generate_vertex_cover_like(size_t num_nodes, size_t extra_arcs, size_t seed) {
  // Conceptually:
  // 1. Generate a *undirected* bipartite graph with num_nodes nodes
  // 2. Connect each node with it's `opposite` s.t. there is matching of size num_nodes/2. Therefore
  //    The min vertex cover is at least num_nodes/2.
  // 3. Randomly add more edges.
  // 4. Turn into a directed graph by back-and-forward edges.
  // The now, the minimum FVS must be a minimum VC.
  assert(num_nodes % 2 == 0);

  Graph graph(num_nodes);

  for (size_t idx = 0; idx < num_nodes; idx++) {
    if (idx % 2 == 0) {
      graph.add_arc_unsafe(idx, idx + 1);
      graph.add_arc_unsafe(idx + 1, idx);
    }
  }

  std::mt19937 gen(static_cast<unsigned int>(seed));

  for (size_t arcs = 0; arcs < extra_arcs;) {
    // We don't want the minimum vertex cover to increase in size, therefore we only allow arcs
    // which cross the bipartition. Since the bipartition is given by even and odd-indexed nodes, we
    // generate one odd and one even index.
    auto from = (gen() % num_nodes / 2) * 2;
    auto to = (gen() % ((num_nodes / 2) - 1)) * 2 + 1;

    if (!graph.is_arc(from, to)) {
      arcs++;

      graph.add_arc_unsafe(from, to);
      graph.add_arc_unsafe(to, from);
    }
  }

  FvsInstance instance(graph);
  instance.set_solution_size(num_nodes / 2);
  instance.permute();
  instance._name = fmt::format("vertex cover like (num_nodes = {}, extra_arcs = {}, seed={})",
                               num_nodes, extra_arcs, seed);

  return instance;
}
FvsInstance generate_random_instance(size_t num_nodes, size_t num_arcs, size_t seed) {
  return gen_random_instance(num_nodes, num_arcs, seed, true);
}

FvsInstance gen_random_instance(size_t num_nodes, size_t num_arcs, size_t seed,
                                bool selfloops_allowed) {
  Graph graph(num_nodes);

  std::mt19937 gen(static_cast<unsigned int>(seed));

  for (size_t arc_count = 0; arc_count < num_arcs;) {

    auto from = gen() % num_nodes;
    auto to = gen() % num_nodes;
    if (!graph.is_arc(from, to) && (selfloops_allowed || from != to)) {
      graph.add_arc_unsafe(from, to);
      arc_count++;
    }
  }

  return FvsInstance(graph);
}
} // namespace algorithms
