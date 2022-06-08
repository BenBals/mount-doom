
include(FetchContent)

### fmt ###
FetchContent_Declare(
        fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG 7.1.3
)
FetchContent_MakeAvailable(fmt)

### spdlog ###
FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.8.5
)
FetchContent_MakeAvailable(spdlog)

### GTest ###
if (ENABLE_TESTING)
    FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG release-1.11.0
    )
    set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
    set(BUILD_GTEST ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    include(GoogleTest)
    enable_testing()
endif ()

### Boost ###
if(DOOM_STATIC)
    set(Boost_USE_STATIC_LIBS ON)
endif()
find_package(Boost COMPONENTS program_options REQUIRED)
