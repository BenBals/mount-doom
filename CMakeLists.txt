# CMake References: https://github.com/Lectem/cpp-boilerplate/blob/master/CMakeLists.txt

cmake_minimum_required(VERSION 3.17)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Do not build in-source. Please remove CMakeCache.txt and the CMakeFiles/ directory. Then build out-of-source.")
endif ()

project(doom
        VERSION 0.1.0
        LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (APPLE)
    add_compile_options(-mmacosx-version-min=10.15)
endif ()

############################
##  Modules and scripts   ##
############################

# Standard CMake modules

include(CTest)                          # Must be called before adding tests but after calling project(). This automatically calls enable_testing() and configures ctest targets when using Make/Ninja
include(CMakeDependentOption)           # This is a really useful scripts that creates options that depends on other options. It can even be used with generator expressions !
include(GNUInstallDirs)                 # This will define the default values for installation directories (all platforms even if named GNU)
include(InstallRequiredSystemLibraries) # Tell CMake that the `install` target needs to install required system libraries (eg: Windows SDK)
include(CMakePackageConfigHelpers)      # Helper to create relocatable packages

# Custom modules and scripts

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake") # Make our cmake scripts available

###############
##  Options  ##
###############

option(ENABLE_TESTING "Should unit tests be compiled." ON)
option(DOOM_STATIC "Should we build a static binary?" OFF)

set(${PROJECT_NAME}_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
        CACHE STRING "Path to install ${PROJECT_NAME} Config*.cmake files to.")

####################
##  Dependencies  ##
####################

include(cmake/dependencies.cmake)

#####################
## Compiler Config ##
#####################
add_compile_options(
        -Wall
        -Wextra
        -Wshadow
        -Wfloat-equal
        -Wconversion
		-Werror
        -Wno-error=conversion
        -Wpedantic
        -Wunused
        -Wuninitialized
        -Wcast-qual
        -Woverloaded-virtual
        -Wredundant-decls
        -Winit-self
        -Wno-unused-function
)

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    add_compile_options(-O0 -g)
ENDIF ()

IF (APPLE)
    # macOS's linker uses different flags all the time
    set(LARGE_STACK_LINKER_FLAGS "LINKER:-stack_size,0x10000000")
ELSE()
    set(LARGE_STACK_LINKER_FLAGS "LINKER:-z,stack-size=0x10000000")
ENDIF()

###############
##  Targets  ##
###############

add_subdirectory(third_party)

# Use the project root to find includes
include_directories(${PROJECT_SOURCE_DIR})
# Make third party libraries available as system libraries to avoid compiler warnings as errors for them
include_directories(SYSTEM ${PROJECT_SOURCE_DIR}/third_party)
add_subdirectory(src)

if (ENABLE_TESTING)
    if (DOOM_STATIC)
        message(WARNING "Disabling tests, as they are incompatible with static linking")
    else()
        add_subdirectory(test)
    endif()
endif ()

