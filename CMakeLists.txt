cmake_minimum_required(VERSION 3.17)

project(jr_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SANDBOX "Build sandbox" ON)

add_subdirectory(api)
add_subdirectory(core)

if(ANDROID)
    add_subdirectory(android)
endif()

if(BUILD_SANDBOX)
    find_package(OpenCV QUIET)

    add_subdirectory(sandbox)
endif()