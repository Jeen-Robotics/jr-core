# Rust middleware bindings with C++ wrapper
#
# Builds:
#   1. Rust static library via Corrosion (workspace crate: middleware)
#   2. CXX bridge target (generated FFI glue + include dirs)
#   3. C++ wrapper library: middleware_rs_cpp
#
# Usage:
#   target_link_libraries(your_target PRIVATE middleware_rs_cpp)

set(JR_MIDDLEWARE_MANIFEST "${PROJECT_SOURCE_DIR}/rust/middleware/Cargo.toml")

if(NOT TARGET middleware)
    corrosion_import_crate(MANIFEST_PATH ${JR_MIDDLEWARE_MANIFEST})
endif()

# CXX bridge target: corrosion_add_cxxbridge handles include path discovery
# across all platforms/configurations.
if(NOT TARGET jr_rust_middleware_cxxbridge)
    corrosion_add_cxxbridge(jr_rust_middleware_cxxbridge
        CRATE middleware
        FILES ffi/bridge.rs
    )
endif()

# Find protobuf for the wrapper
find_package(Protobuf REQUIRED)

# C++ wrapper library
add_library(middleware_rs_cpp STATIC
    src/wrapper.cpp
)

target_include_directories(middleware_rs_cpp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against Rust library and dependencies
target_link_libraries(middleware_rs_cpp
    PUBLIC
        protobuf::libprotobuf
    PRIVATE
        jr_rust_middleware_cxxbridge  # CXX bridge: generated glue + include dirs for bridge.rs.h
        middleware                     # Rust static lib from Corrosion
)

# C++17 required for std::optional
target_compile_features(middleware_rs_cpp PUBLIC cxx_std_17)

# Example executable (optional, only built with tests)
if(BUILD_TESTS)
    find_package(jr_msgs CONFIG REQUIRED)

    add_executable(middleware_rs_example
        examples/basic_usage.cpp
    )

    target_link_libraries(middleware_rs_example
        PRIVATE
            middleware_rs_cpp
            jr_msgs::jr_msgs
    )
endif()
