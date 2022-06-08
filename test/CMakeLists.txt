option(ALWAYS_ENABLE_EXPENSIVE_TESTS "" OFF)

target_compile_definitions(datastructures PUBLIC TEST)
add_executable(datastructures_test)
target_sources(datastructures_test PRIVATE datastructures_test.cpp)
target_link_libraries(datastructures_test
        PRIVATE algorithms datastructures gtest gmock gtest_main executable_helpers helpers
        )
gtest_discover_tests(datastructures_test)

target_compile_definitions(algorithms PUBLIC TEST)
add_executable(algorithms_test)
target_sources(algorithms_test PRIVATE algorithms_test.cpp)
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" OR ALWAYS_ENABLE_EXPENSIVE_TESTS)
    target_sources(algorithms_test PRIVATE algorithms_test_heavy.cpp)
endif()

target_link_libraries(algorithms_test
        PRIVATE algorithms datastructures gtest gmock gtest_main executable_helpers helpers
        )
gtest_discover_tests(algorithms_test)
