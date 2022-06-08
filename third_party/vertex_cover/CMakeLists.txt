project(vertex-cover C CXX)

add_library(libkaffpa2)
set(LIBKAFFPA_SOURCE_FILES
        extern/KaHIP/interface/kaHIP_interface.cpp
        extern/KaHIP/lib/data_structure/graph_hierarchy.cpp
        extern/KaHIP/lib/algorithms/strongly_connected_components.cpp
        extern/KaHIP/lib/algorithms/topological_sort.cpp
        extern/KaHIP/lib/algorithms/push_relabel.cpp
        extern/KaHIP/lib/io/graph_io.cpp
        extern/KaHIP/lib/tools/quality_metrics.cpp
        extern/KaHIP/lib/tools/random_functions.cpp
        extern/KaHIP/lib/tools/graph_extractor.cpp
        extern/KaHIP/lib/tools/misc.cpp
        extern/KaHIP/lib/tools/partition_snapshooter.cpp
        extern/KaHIP/lib/partition/graph_partitioner.cpp
        extern/KaHIP/lib/partition/w_cycles/wcycle_partitioner.cpp
        extern/KaHIP/lib/partition/coarsening/coarsening.cpp
        extern/KaHIP/lib/partition/coarsening/contraction.cpp
        extern/KaHIP/lib/partition/coarsening/edge_rating/edge_ratings.cpp
        extern/KaHIP/lib/partition/coarsening/matching/matching.cpp
        extern/KaHIP/lib/partition/coarsening/matching/random_matching.cpp
        extern/KaHIP/lib/partition/coarsening/matching/gpa/path.cpp
        extern/KaHIP/lib/partition/coarsening/matching/gpa/gpa_matching.cpp
        extern/KaHIP/lib/partition/coarsening/matching/gpa/path_set.cpp
        extern/KaHIP/lib/partition/coarsening/clustering/node_ordering.cpp
        extern/KaHIP/lib/partition/coarsening/clustering/size_constraint_label_propagation.cpp
        extern/KaHIP/lib/partition/initial_partitioning/initial_partitioning.cpp
        extern/KaHIP/lib/partition/initial_partitioning/initial_partitioner.cpp
        extern/KaHIP/lib/partition/initial_partitioning/initial_partition_bipartition.cpp
        extern/KaHIP/lib/partition/initial_partitioning/initial_refinement/initial_refinement.cpp
        extern/KaHIP/lib/partition/initial_partitioning/bipartition.cpp
        extern/KaHIP/lib/partition/uncoarsening/uncoarsening.cpp
        extern/KaHIP/lib/partition/uncoarsening/separator/vertex_separator_algorithm.cpp
        extern/KaHIP/lib/partition/uncoarsening/separator/vertex_separator_flow_solver.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/cycle_improvements/greedy_neg_cycle.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/cycle_improvements/problem_factory.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/cycle_improvements/augmented_Qgraph.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/mixed_refinement.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/label_propagation_refinement/label_propagation_refinement.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/refinement.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/2way_fm_refinement/two_way_fm.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/flow_refinement/two_way_flow_refinement.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/flow_refinement/boundary_bfs.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/flow_refinement/flow_solving_kernel/cut_flow_problem_solver.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/flow_refinement/most_balanced_minimum_cuts/most_balanced_minimum_cuts.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/quotient_graph_refinement.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/complete_boundary.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/partial_boundary.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/quotient_graph_scheduling/quotient_graph_scheduling.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/quotient_graph_scheduling/simple_quotient_graph_scheduler.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/quotient_graph_scheduling/active_block_quotient_graph_scheduler.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/kway_graph_refinement/kway_graph_refinement.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/kway_graph_refinement/kway_graph_refinement_core.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/kway_graph_refinement/kway_graph_refinement_commons.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/cycle_improvements/augmented_Qgraph_fabric.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/cycle_improvements/advanced_models.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/kway_graph_refinement/multitry_kway_fm.cpp
        extern/KaHIP/lib/algorithms/cycle_search.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/cycle_improvements/cycle_refinement.cpp
        extern/KaHIP/lib/partition/uncoarsening/refinement/tabu_search/tabu_search.cpp)

target_include_directories(libkaffpa2
        INTERFACE
        extern/KaHIP
        extern/KaHIP/interface
        extern/KaHIP/lib
        extern/KaHIP/lib/tools
        extern/KaHIP/lib/io
        extern/KaHIP/lib/partition
        extern/KaHIP/lib/parallel_mh
        extern/KaHIP/lib/data_structure
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/flow_refinement
        PRIVATE
        extern/KaHIP
        extern/KaHIP/lib
        extern/KaHIP/lib/tools
        extern/KaHIP/lib/io
        extern/KaHIP/lib/partition
        extern/KaHIP/lib/parallel_mh
        extern/KaHIP/lib/data_structure
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/flow_refinement)

target_link_libraries(libkaffpa2 PRIVATE)
target_sources(libkaffpa2 PRIVATE ${LIBKAFFPA_SOURCE_FILES})


get_target_property(LIBKAFFPA_COMPILE_FLAGS libkaffpa2 COMPILE_OPTIONS)
if (LIBKAFFPA_COMPILE_FLAGS STREQUAL "LIBKAFFPA_COMPILE_FLAGS-NOTFOUND")
	set(LIBKAFFPA_COMPILE_FLAGS "")
endif()

list(REMOVE_ITEM LIBKAFFPA_COMPILE_FLAGS -Wconversion)
list(REMOVE_ITEM LIBKAFFPA_COMPILE_FLAGS -Wshadow)
list(REMOVE_ITEM LIBKAFFPA_COMPILE_FLAGS -Wtype-limits)
list(REMOVE_ITEM LIBKAFFPA_COMPILE_FLAGS -Wunused-parameter)
list(REMOVE_ITEM LIBKAFFPA_COMPILE_FLAGS -Wfloat-equal)
list(REMOVE_ITEM LIBKAFFPA_COMPILE_FLAGS -Werror)
list(APPEND LIBKAFFPA_COMPILE_FLAGS -Wno-unused-private-field)
list(APPEND LIBKAFFPA_COMPILE_FLAGS -Wno-vla-extension)
list(APPEND LIBKAFFPA_COMPILE_FLAGS -Wno-unused-parameter)
list(APPEND LIBKAFFPA_COMPILE_FLAGS -Wno-sign-compare)
list(APPEND LIBKAFFPA_COMPILE_FLAGS -Wno-unused-but-set-variable)

set_target_properties(libkaffpa2 PROPERTIES COMPILE_OPTIONS "${LIBKAFFPA_COMPILE_FLAGS}")

add_library(vertex-cover-solver)

target_include_directories(vertex-cover-solver
        INTERFACE
        lib
        lib/mis

        PRIVATE
        lib
        lib/data_structure
        lib/data_structure/priority_queues
        lib/mis
        lib/mis/ils
        lib/mis/initial_mis
        lib/mis/kernel
        lib/mis/evolutionary
        lib/tools
        extern/KaHIP
        extern/KaHIP/app
        extern/KaHIP/interface
        extern/KaHIP/lib
        extern/KaHIP/lib/algorithms
        extern/KaHIP/lib/data_structure
        extern/KaHIP/lib/data_structure/matrix
        extern/KaHIP/lib/data_structure/priority_queues
        extern/KaHIP/lib/io
        extern/KaHIP/lib/parallel_mh
        extern/KaHIP/lib/parallel_mh/exchange
        extern/KaHIP/lib/parallel_mh/galinier_combine
        extern/KaHIP/lib/partition
        extern/KaHIP/lib/partition/uncoarsening/refinement/quotient_graph_refinement/flow_refinement
        extern/KaHIP/lib/tools
        extern/cliqueSolver
)


set(SOURCES
        lib/tools/mis_log.cpp
        lib/mis/exact_mis.cpp
        lib/mis/ils/ils.cpp
        lib/mis/ils/local_search.cpp
        lib/mis/initial_mis/greedy_mis.cpp
        lib/mis/initial_mis/greedy_vertex.cpp
        lib/mis/initial_mis/random_mis.cpp
        lib/mis/initial_mis/initial_mis.cpp
        lib/mis/evolutionary/population_mis.cpp
        lib/data_structure/mis_permutation.cpp
        lib/data_structure/candidate_list.cpp
        lib/data_structure/operation_log.cpp
        lib/data_structure/priority_queues/bucket_array.cpp
        lib/mis/kernel/branch_and_reduce_algorithm.cpp
        lib/mis/kernel/modified.cpp)

target_sources(vertex-cover-solver
        INTERFACE
        vertex_cover.h
		configuration_mis.h
        PRIVATE
        ${SOURCES})

target_link_libraries(vertex-cover-solver
        libkaffpa2)

get_target_property(VCSOLVER_LIB_COMPILE_FLAGS vertex-cover-solver COMPILE_OPTIONS)
if (VCSOLVER_LIB_COMPILE_FLAGS STREQUAL "VCSOLVER_LIB_COMPILE_FLAGS-NOTFOUND")
	set(VCSOLVER_LIB_COMPILE_FLAGS "")
endif()
list(REMOVE_ITEM VCSOLVER_LIB_COMPILE_FLAGS -Wconversion)
list(REMOVE_ITEM VCSOLVER_LIB_COMPILE_FLAGS -Wshadow)
list(REMOVE_ITEM VCSOLVER_LIB_COMPILE_FLAGS -Wtype-limits)
list(REMOVE_ITEM VCSOLVER_LIB_COMPILE_FLAGS -Wunused-parameter)
list(REMOVE_ITEM VCSOLVER_LIB_COMPILE_FLAGS -Werror)
list(REMOVE_ITEM VCSOLVER_LIB_COMPILE_FlAGS -Wunused-variable)
list(REMOVE_ITEM VCSOLVER_LIB_COMPILE_FlAGS -Wunused-but-set-variable)
list(REMOVE_ITEM VCSOLVER_LIB_COMPILE_FlAGS -Wcast-qual)
list(APPEND VCSOLVER_LIB_COMPILE_FLAGS -Wno-type-limits)
list(APPEND VCSOLVER_LIB_COMPILE_FLAGS -Wno-vla)
list(APPEND VCSOLVER_LIB_COMPILE_FLAGS -Wno-vla-extension)
list(APPEND VCSOLVER_LIB_COMPILE_FLAGS -Wno-unused-parameter)
list(APPEND VCSOLVER_LIB_COMPILE_FLAGS -Wno-unused-variable)
list(APPEND VCSOLVER_LIB_COMPILE_FLAGS -Wno-cast-qual)
list(APPEND VCSOLVER_LIB_COMPILE_FLAGS -Wno-sign-compare)

set_target_properties(vertex-cover-solver PROPERTIES COMPILE_OPTIONS "${VCSOLVER_LIB_COMPILE_FLAGS}")
