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

# ReRun visualization SDK (uses Arrow from Conan)
if(BUILD_SANDBOX AND NOT ANDROID)
    set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF CACHE BOOL "Use system Arrow")
    find_package(Arrow REQUIRED)
    
    include(FetchContent)
    FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
    FetchContent_MakeAvailable(rerun_sdk)
endif()

add_subdirectory(api)
add_subdirectory(core)

if(BUILD_SANDBOX)
    find_package(OpenCV REQUIRED)

    add_subdirectory(sandbox)
endif()

if(ANDROID)
    add_subdirectory(android)
endif()