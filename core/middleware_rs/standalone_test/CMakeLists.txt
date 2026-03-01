# Standalone build test for middleware_rs
# Usage:
#   mkdir build && cd build
#   cmake .. && make

cmake_minimum_required(VERSION 3.17)
project(middleware_rs_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Fetch Corrosion
include(FetchContent)
FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    GIT_TAG v0.5.1
)
FetchContent_MakeAvailable(Corrosion)

# Find protobuf
find_package(Protobuf REQUIRED)

# Import Rust crate from parent directory
set(MIDDLEWARE_RS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
corrosion_import_crate(MANIFEST_PATH ${MIDDLEWARE_RS_DIR}/Cargo.toml)

# CXX bridge headers location
set(CXX_BRIDGE_DIR ${MIDDLEWARE_RS_DIR}/target/cxxbridge)

# C++ wrapper library
add_library(middleware_rs_cpp STATIC
    ${MIDDLEWARE_RS_DIR}/src/wrapper.cpp
)

target_include_directories(middleware_rs_cpp
    PUBLIC
        ${MIDDLEWARE_RS_DIR}/include
    PRIVATE
        ${CXX_BRIDGE_DIR}
)

target_link_libraries(middleware_rs_cpp
    PUBLIC
        protobuf::libprotobuf
    PRIVATE
        middleware_rs
)

add_dependencies(middleware_rs_cpp cargo-build_middleware_rs)

target_compile_features(middleware_rs_cpp PUBLIC cxx_std_17)

# Simple test executable (no proto messages, just API check)
add_executable(api_test
    api_test.cpp
)

target_link_libraries(api_test
    PRIVATE
        middleware_rs_cpp
)
