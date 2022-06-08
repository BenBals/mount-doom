#include "vertex_cover.h"

// this suppresses warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wconversion"
#ifndef __clang__ // clang does not support this warning
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif
#include <numvc.hpp>
// this reenables warnings
#pragma GCC diagnostic pop

#include <vc-satreduce/include/bnb.hpp>
#include <vertex_cover/configuration_mis.h>
#include <vertex_cover/lib/mis/kernel/branch_and_reduce_algorithm.h>
#include <vertex_cover/vertex_cover.h>

namespace algorithms {
namespace {

Flags common_flags() {
  Flags flags{};
  flags.packing = true;
  flags.use_funnel = false;
  flags.constraint_totalizer = false;
  flags.use_lp_reduction = true;
  return flags;
}

std::optional<std::vector<bool>> short_satreduce_kernel_solve(std::vector<std::vector<int>> &&adj) {
  // convert to satreduce
  auto graph = ::Graph(adj);

  helpers::reset_timeout();
  helpers::set_timeout(10, 0);
  auto solver = vc_bnb::Solver(graph);
  solver.flags = common_flags();

  auto solved = solver.solve();
  helpers::reset_timeout();

  if (solved)
    return std::make_optional(std::move(solver.best_solution.picked_vertices));
  return {};
}

std::optional<std::vector<bool>>
exhaustive_satreduce_kernel_solve(std::vector<std::vector<int>> &&adj) {
  // convert to satreduce
  auto graph = ::Graph(adj);

  helpers::reset_timeout();

  auto satreduce_solver = vc_bnb::Solver(graph);
  satreduce_solver.flags = common_flags();
  satreduce_solver.flags.ub = ub_algorithm::numvc;

  auto solved = satreduce_solver.solve();
  if (!solved) {
    spdlog::critical("exhaustive satreduce did not solve");
    exit(1);
  }
  return satreduce_solver.best_solution.picked_vertices;
}
std::optional<std::vector<bool>> numvc_kernel_solve(std::vector<std::vector<int>> &&adj) {
  std::vector<std::pair<int, int>> edges;
  for (size_t idx = 0; idx < adj.size(); idx++) {
    for (auto other : adj[idx]) {
      if (idx <= static_cast<size_t>(other))
        edges.emplace_back(static_cast<int>(idx), other);
    }
  }

  auto numvc = libmvc::NuMVC(edges, static_cast<int>(adj.size()), 0,
                             std::chrono::milliseconds(std::min(adj.size(), 10000ul)));
  numvc.cover_LS();

  const auto vertex_cover_char_flags = numvc.get_cover_as_flaglist();
  std::vector<bool> vertex_cover_flags(adj.size());
  for (size_t idx = 0; idx < vertex_cover_flags.size(); idx++) {
    vertex_cover_flags[idx] = vertex_cover_char_flags[idx];
  }
  return std::make_optional(std::move(vertex_cover_flags));
}

template <class F>
requires(std::invocable<F, std::vector<std::vector<int>> &&>
             &&std::is_same<std::optional<std::vector<bool>>,
                            std::invoke_result_t<F, std::vector<std::vector<int>> &&>>::value)
    std::optional<std::vector<bool>> solve_mis_on_undirected_edges(
        const datastructures::Graph &graph, F &&try_solve_vc_kernel) {
  // convert graph representation to WeGoYouCovered
  std::vector<std::vector<int>> initial_adj(graph.size());
  for (auto &node : graph._nodes) {
    initial_adj[node.index()] = {node.undirected_edges().begin(), node.undirected_edges().end()};
    std::sort(initial_adj[node.index()].begin(), initial_adj[node.index()].end());
  }

  // kernelize graph using WeGotYouCovered
  auto reduction_algorithm =
      branch_and_reduce_algorithm(initial_adj, static_cast<int>(initial_adj.size()));
  reduction_algorithm.reduce();

  graph_access vc_kernel;
  std::vector<NodeID> vc_kernel_reverse_mapping(reduction_algorithm.number_of_nodes_remaining());
  reduction_algorithm.convert_adj_lists(vc_kernel, vc_kernel_reverse_mapping);
  spdlog::info("\t\t\treduced graph size {}", vc_kernel.number_of_nodes());

  // solve vertex cover using other solver
  std::vector<bool> finalSolution(graph.size(), false);
  if (vc_kernel.number_of_nodes() > 0) {
    std::vector<std::vector<int>> kernel_adj(vc_kernel.number_of_nodes());
    forall_nodes(vc_kernel, node) {
      forall_out_edges(vc_kernel, edge, node) {
        auto other = vc_kernel.getEdgeTarget(edge);
        if (other <= node) {
          kernel_adj[node].push_back(static_cast<int>(other));
          kernel_adj[other].push_back(static_cast<int>(node));
        }
      }
      endfor;
    }
    endfor;

    auto kernel_solution_opt = try_solve_vc_kernel(std::move(kernel_adj));
    if (!kernel_solution_opt)
      return {};

    // Propagate solution to preliminary kernel
    for (size_t i = 0; i < kernel_solution_opt->size(); ++i)
      // need to invert since sub solver gives vc
      finalSolution[vc_kernel_reverse_mapping[i]] = !kernel_solution_opt.value()[i];
    spdlog::info("\t\t\tVC on reduced graph {}",
                 std::count(kernel_solution_opt->begin(), kernel_solution_opt->end(), true));
  }

  // extend to original graph and return
  reduction_algorithm.extend_finer_is(finalSolution);
  return std::make_optional(std::move(finalSolution));
}
std::vector<bool> solve_mis_on_undirected_edges_exact(const datastructures::Graph &graph) {
  auto mis_opt = solve_mis_on_undirected_edges(graph, short_satreduce_kernel_solve);
  if (mis_opt)
    return mis_opt.value();

  return solve_mis_on_undirected_edges(graph, exhaustive_satreduce_kernel_solve).value();
}
} // namespace
VertexCoverSolver::VertexCoverSolver(Solver *inner) : DecoratingSolver(inner) {}
std::optional<std::vector<size_t>> VertexCoverSolver::solve(FvsInstance &&instance) {
  if (!instance.is_vc_instance())
    return solve_with_inner(std::move(instance));

  auto mis = solve_mis_on_undirected_edges_exact(instance._graph);

  for (size_t idx = 0; idx < instance._graph.size(); idx++) {
    if (!mis[idx]) {
      instance.take_node_into_partial_solution(idx);
    }
  }

  if (instance._partial_solution.size() < instance._upper_bound)
    return {std::move(instance._partial_solution)};

  return {};
}
UnderlyingVertexCoverSolver::UnderlyingVertexCoverSolver(Solver *inner) : DecoratingSolver(inner) {}
std::optional<std::vector<size_t>> UnderlyingVertexCoverSolver::solve(FvsInstance &&instance) {
  instance.collect_self_loops_into_partial_fvs();

  auto graph_copy = instance._graph;
  auto mis = solve_mis_on_undirected_edges_exact(graph_copy);
  spdlog::info("solved vc on underlying undirected graph of size {}", graph_copy.size());

  size_t vertex_cover_size = 0;
  for (size_t idx = 0; idx < graph_copy.size(); idx++) {
    if (!mis[idx]) {
      vertex_cover_size++;
      graph_copy.isolate_node(idx);
    }
  }
  spdlog::info("\tvc has size {}", vertex_cover_size);

  if (instance.partial_solution_size() + vertex_cover_size >= instance._upper_bound) {
    spdlog::info("\tpruned by vc lower bound");
    return {};
  }

  if (!graph_copy.is_acyclic()) {
    spdlog::info("\tat least {} disjoint cycles remaining",
                 graph_copy.greedy_count_disjoint_cycles());
    // in an SCC of size k we only need to take at most k-1 additional nodes
    // Therefore we need to take at most n - num_sccs additional nodes
    auto [num_sccs, _scc_colors] = graph_copy.strongly_connected_components();
    instance._upper_bound = std::min(instance._upper_bound,
                                     instance.partial_solution_size() + vertex_cover_size +
                                         graph_copy.size() - num_sccs + 1 /*strict upper bound */);
    spdlog::info("\tupper bound is now {}", instance._upper_bound);
    return solve_with_inner(std::move(instance));
  }

  spdlog::info("\tvc did solve DFVS instance. YEAH.");
  for (size_t idx = 0; idx < instance._graph.size(); idx++) {
    if (!mis[idx]) {
      instance.take_node_into_partial_solution(idx);
    }
  }
  return {std::move(instance._partial_solution)};
}
std::optional<std::vector<size_t>> CycleCliqueVertexCoverSolver::solve(FvsInstance &&instance) {
  instance.collect_self_loops_into_partial_fvs();

  // we will first have the real nodes and then the clique nodes
  auto graph_with_cliques = instance._graph;
  struct Cycle {
    size_t representive_node_index = SIZE_MAX;
    bool atleast_one_in_mis = false;
  };
  std::vector<Cycle> cycles;
  // the index is shifted by instance._graph.size();
  std::vector<size_t> cycle_of_additional_node;
  spdlog::info("\tinstance size {}", instance._graph.size());

  auto initial_solver_fun = numvc_kernel_solve;
  while (true) {
    spdlog::info("\t\tvc instance size {}", graph_with_cliques.size());
    auto mis_opt = solve_mis_on_undirected_edges(graph_with_cliques, initial_solver_fun);
    auto cur_mis = mis_opt.value();

    auto convert_mis_to_dfvs = [&](const std::vector<bool> &mis) {
      // look whether the current solution solves the DFVS
      auto remaining_graph = instance._graph;

      std::vector<size_t> additional_dfvs_nodes;
      for (size_t idx = 0; idx < remaining_graph.size(); idx++) {
        if (!mis[idx]) {
          remaining_graph.isolate_node(idx);
          additional_dfvs_nodes.push_back(idx);
        }
      }

      // check for cliques that were taken completely in vertex cover
      // so check whether any node is taken in cur_mis
      for (auto &cycle : cycles)
        cycle.atleast_one_in_mis = false;
      for (size_t idx = remaining_graph.size(); idx < graph_with_cliques.size(); idx++) {
        auto shifted_node_idx = idx - remaining_graph.size();
        auto cycle_idx = cycle_of_additional_node[shifted_node_idx];
        cycles[cycle_idx].atleast_one_in_mis |= mis[idx];
      }
      // take representative of cycles that are completely in VC/not at all in MIS
      for (const auto &cycle : cycles) {
        if (!cycle.atleast_one_in_mis) {
          remaining_graph.isolate_node(cycle.representive_node_index);
          additional_dfvs_nodes.push_back(cycle.representive_node_index);
        }
      }

      return std::make_pair(additional_dfvs_nodes, remaining_graph);
    };

    auto [additional_dfvs_nodes, remaining_graph] = convert_mis_to_dfvs(cur_mis);
    // this can happen if we took wrong nodes into the partial solution before calling this solver
    if (instance.partial_solution_size() + additional_dfvs_nodes.size() >= instance._upper_bound) {
      spdlog::info("\t\tlower bound of {} found, but upper bound is {}",
                   instance.partial_solution_size() + additional_dfvs_nodes.size(),
                   instance._upper_bound);
      return {};
    }

    bool is_final_solution = false;
    if (remaining_graph.is_acyclic()) {
      is_final_solution = true; // preliminary
      spdlog::info("\t\tfound dfvs solution of size {}, with upper bound {}",
                   instance.partial_solution_size() + additional_dfvs_nodes.size(),
                   instance._upper_bound);
    }

    if (is_final_solution || helpers::is_timeout_hit()) {
      spdlog::info("start greedy with already {} nodes chosen", additional_dfvs_nodes.size());
      auto remaining_instance = FvsInstance(remaining_graph);
      auto greedy_solver = algorithms::MaxByDegreeHeuristicSolver();

      spdlog::info("finding initial solution");
      auto greedy_initial_solution =
          greedy_solver.greedily_find_initial_solution(std::move(remaining_instance));
      spdlog::info("found initial solution of size {}", greedy_initial_solution.size());

      spdlog::info("minimizing union solution of solutions");
      std::vector<size_t> solution_to_minimize;
      for (auto ccvc_solution_node_idx : additional_dfvs_nodes) {
        solution_to_minimize.push_back(instance._original_indices[ccvc_solution_node_idx]);
      }
      for (auto new_dfvs_node_idx : greedy_initial_solution) {
        // the original indices in `remaining_instance` are by default the indices in
        // `remaining_graph` + 1
        solution_to_minimize.push_back(instance._original_indices[new_dfvs_node_idx - 1]);
      }
      auto greedy_minimal_solution1 =
          greedy_solver.make_solution_minimal(instance, solution_to_minimize);

      // try in reverse order as well
      solution_to_minimize.clear();
      for (auto new_dfvs_node_idx : greedy_initial_solution) {
        // the original indices in `remaining_instance` are by default the indices in
        // `remaining_graph` + 1
        solution_to_minimize.push_back(instance._original_indices[new_dfvs_node_idx - 1]);
      }
      for (auto ccvc_solution_node_idx : additional_dfvs_nodes) {
        solution_to_minimize.push_back(instance._original_indices[ccvc_solution_node_idx]);
      }
      auto greedy_minimal_solution2 =
          greedy_solver.make_solution_minimal(instance, solution_to_minimize);

      auto greedy_minimal_solution =
          greedy_minimal_solution1.size() <= greedy_minimal_solution2.size()
              ? greedy_minimal_solution1
              : greedy_minimal_solution2;

      spdlog::info("minimized solutions to size {}", greedy_minimal_solution.size());
      spdlog::info("end greedy");

      for (auto additional_solution_original_index : greedy_minimal_solution) {
        instance._partial_solution.push_back(additional_solution_original_index);
      }

      return {std::move(instance._partial_solution)};
    }

    // otherwise we have to cover some additional cycles
    auto new_cycles = remaining_graph.greedy_find_disjoint_cycles();
    spdlog::info("\t\tpartial dfvs size {}, remaining disjoint cycles {}",
                 instance.partial_solution_size() + additional_dfvs_nodes.size(),
                 new_cycles.size());
    for (const auto &cycle : new_cycles) {
      helpers::assert_and_log(!cycle.empty(), "a cycle can not be empty");

      size_t highest_degree_node_idx = cycle.front();
      for (auto cycle_idx : cycle) {
        if (remaining_graph[cycle_idx].all_edges().size() >= remaining_graph[highest_degree_node_idx].all_edges().size()) {
          highest_degree_node_idx = cycle_idx;
        }
      }

      cycles.emplace_back(Cycle{.representive_node_index = cycle.front()});

      // this relies upon that created nodes are added at the end
      auto clique_idx_start = graph_with_cliques.size();
      for (auto cycle_node : cycle) {
        size_t clique_node = graph_with_cliques.create_node();
        cycle_of_additional_node.push_back(cycles.size() - 1);
        graph_with_cliques.add_arc(cycle_node, clique_node);
        graph_with_cliques.add_arc(clique_node, cycle_node);
      }

      // make cliques nodes real cliques
      for (size_t first_clique_node = clique_idx_start;
           first_clique_node < graph_with_cliques.size(); first_clique_node++) {
        for (size_t second_clique_node = first_clique_node + 1;
             second_clique_node < graph_with_cliques.size(); second_clique_node++) {
          graph_with_cliques.add_arc(first_clique_node, second_clique_node);
          graph_with_cliques.add_arc(second_clique_node, first_clique_node);
        }
      }
    }
  }
}
} // namespace algorithms
