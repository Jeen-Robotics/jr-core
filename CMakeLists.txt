cmake_minimum_required(VERSION 3.17)

project(jr_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SANDBOX "Build sandbox" ON)
option(BUILD_TESTS "Build tests" OFF)

if(BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
endif()

# Corrosion for Rust integration (required for middleware)
if(NOT ANDROID)
    include(FetchContent)
    FetchContent_Declare(
        Corrosion
        GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
        GIT_TAG v0.5.1
    )
    FetchContent_MakeAvailable(Corrosion)
endif()

# Check for optional dependencies
find_package(OpenCV QUIET COMPONENTS core)
find_package(mcap CONFIG QUIET)
find_package(Arrow QUIET)

# Track if full API can be built
set(BUILD_FULL_API OFF)

if(OpenCV_FOUND AND mcap_FOUND)
    set(BUILD_FULL_API ON)
endif()

# ReRun visualization SDK (uses Arrow from Conan)
if(BUILD_SANDBOX AND NOT ANDROID AND Arrow_FOUND)
    set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF CACHE BOOL "Use system Arrow")
    
    include(FetchContent)
    FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
    FetchContent_MakeAvailable(rerun_sdk)
endif()

# API module (requires bag, nodes)
if(BUILD_FULL_API)
    add_subdirectory(api)
else()
    message(STATUS "Skipping api module (missing dependencies)")
endif()

add_subdirectory(core)

if(BUILD_SANDBOX AND OpenCV_FOUND)
    add_subdirectory(sandbox)
elseif(BUILD_SANDBOX)
    message(STATUS "Skipping sandbox (OpenCV not found)")
endif()

if(ANDROID)
    add_subdirectory(android)
endif()
