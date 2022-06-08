add_library(libmvc INTERFACE)
target_include_directories(libmvc INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(libmvc INTERFACE spdlog)