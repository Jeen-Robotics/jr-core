cmake_minimum_required(VERSION 3.17)

project(jr_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SANDBOX "Build sandbox" ON)
option(BUILD_TESTS "Build tests" OFF)
set(JR_MIDDLEWARE_DEFAULT_BACKEND "rust" CACHE STRING "Default middleware backend: rust or cpp")
set_property(CACHE JR_MIDDLEWARE_DEFAULT_BACKEND PROPERTY STRINGS rust cpp)
string(TOLOWER "${JR_MIDDLEWARE_DEFAULT_BACKEND}" JR_MIDDLEWARE_DEFAULT_BACKEND)

if(NOT JR_MIDDLEWARE_DEFAULT_BACKEND STREQUAL "rust" AND
   NOT JR_MIDDLEWARE_DEFAULT_BACKEND STREQUAL "cpp")
    message(FATAL_ERROR "JR_MIDDLEWARE_DEFAULT_BACKEND must be 'rust' or 'cpp'")
endif()

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

# ReRun visualization SDK (uses Arrow from Conan)
if(BUILD_SANDBOX AND NOT ANDROID)
    set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF CACHE BOOL "Use system Arrow")
    find_package(Arrow REQUIRED)
    
    include(FetchContent)
    # Build only ReRun SDK as shared to avoid pulling Rust runtime symbols
    # directly into final executables.
    set(_JR_PREV_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS ON)
    FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
    FetchContent_MakeAvailable(rerun_sdk)
    set(BUILD_SHARED_LIBS ${_JR_PREV_BUILD_SHARED_LIBS})
    unset(_JR_PREV_BUILD_SHARED_LIBS)
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
