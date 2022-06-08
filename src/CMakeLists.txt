# TODO This structure is pretty horrible as of now (we should not have one library target per source file).
# Let's change this as soon as people have pushed their stuff into this branch.

if (DOOM_STATIC)
    set(DOOM_STATIC_FLAG "-static")
endif ()

add_executable(doom)
target_sources(doom PRIVATE doom.cpp license.h)
target_link_libraries(doom PUBLIC ${DOOM_STATIC_FLAG} PRIVATE
        Boost::program_options
        Boost::headers
        spdlog::spdlog
        executable_helpers
        helpers
        datastructures
        algorithms
        INTERFACE ${LARGE_STACK_LINKER_FLAGS}
        )

add_library(datastructures)
target_include_directories(datastructures
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(datastructures
        PUBLIC
        fmt
        spdlog::spdlog
        PRIVATE
        helpers
        Boost::headers
        )
target_sources(datastructures
        INTERFACE
        datastructures.h
        datastructures/fvs_instance.h
        datastructures/fvs_instance_stats.h
        datastructures/graph.h
        datastructures/graph_io.h
        datastructures/cycle_flow_graph.h
        datastructures/partial_solution.h
        PRIVATE
        datastructures/graph.cpp
        datastructures/graph_io.cpp
        datastructures/fvs_instance_stats.cpp
        datastructures/fvs_instance.cpp
        datastructures/fvs_instance_reductions.cpp
        datastructures/cycle_flow_graph.cpp
        datastructures/partial_solution.cpp
        )

add_library(algorithms)
target_include_directories(algorithms
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(algorithms
        PUBLIC
        fmt
        spdlog::spdlog
        datastructures
        vertex-cover-solver
        vc-satreduce
        cadical
        libmvc
        PRIVATE
        Boost::headers
        helpers
        )
target_sources(algorithms
        INTERFACE
        algorithms.h
        algorithms/fvs_generation.h
        algorithms/solvers.h
        algorithms/solvers/base.h
        algorithms/solvers/base_impl.h
        algorithms/solvers/branching.h
        algorithms/solvers/branching_impl.h
        algorithms/solvers/brute_forcing.h
        algorithms/solvers/challenge.h
        algorithms/solvers/articulation_point.h
        algorithms/solvers/greedy.h
        algorithms/solvers/greedy_impl.h
        algorithms/solvers/reducing.h
        algorithms/solvers/scc_splitting.h
        algorithms/solvers/trivial_instance.h
        algorithms/solvers/vertex_cover.h
        helpers.h
        PRIVATE
        algorithms/fvs_generation.cpp
        algorithms/solvers/base.cpp
        algorithms/solvers/branching.cpp
        algorithms/solvers/brute_forcing.cpp
        algorithms/solvers/challenge.cpp
        algorithms/solvers/articulation_point.cpp
        algorithms/solvers/greedy.cpp
        algorithms/solvers/reducing.cpp
        algorithms/solvers/scc_splitting.cpp
        algorithms/solvers/trivial_instance.cpp
        algorithms/solvers/vertex_cover.cpp
        )


add_library(executable_helpers)
target_link_libraries(executable_helpers PRIVATE spdlog::spdlog)
target_sources(executable_helpers
        INTERFACE
        executable_helpers.h
        PRIVATE
        executable_helpers.cpp
        )
target_include_directories(executable_helpers
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        )

if (NOT APPLE)
    set(RT_LIBRARY rt)
endif()

add_library(helpers)
target_link_libraries(helpers PRIVATE
    spdlog::spdlog
    Boost::headers
    vc-satreduce
    ${RT_LIBRARY}
    )
target_sources(helpers
        INTERFACE
        helpers.h
        PRIVATE
        helpers.cpp
        )
target_include_directories(helpers
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        )
