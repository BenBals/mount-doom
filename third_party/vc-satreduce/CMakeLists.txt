add_library(vc-satreduce)
target_compile_options(vc-satreduce PRIVATE -w)
target_include_directories(vc-satreduce SYSTEM
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        )
target_link_libraries(vc-satreduce
        PUBLIC
        cadical
        PRIVATE
        libmvc
        Boost::program_options
        Boost::headers
        )
target_sources(vc-satreduce
        PRIVATE
        src/algorithms.cpp
        src/bnb.cpp
        src/graph.cpp
        src/input-reader.cpp
        src/reductions.cpp
        )

add_executable(vc-satreduce-exec)
target_link_libraries(vc-satreduce-exec
        Boost::program_options
        Boost::headers
        cadical
        vc-satreduce
        )
target_sources(vc-satreduce-exec
        PRIVATE
        src/main.cpp
        )
