cmake_minimum_required(VERSION 3.17)

project(jr_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# NOTE: Actually should be set as an external variable
set(BUILD_SHARED_LIBS ON)

option(BUILD_SANDBOX "Build sandbox" ON)

# TODO: Make it optional?!
find_package(OpenCV REQUIRED)

add_subdirectory(core)

if (BUILD_SANDBOX)
    add_subdirectory(sandbox)
endif()