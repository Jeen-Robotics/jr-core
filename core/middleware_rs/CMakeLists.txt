# Rust middleware with C++ wrapper
#
# Builds:
#   1. Rust static library via Corrosion
#   2. CXX bridge target (generated FFI glue + include dirs)
#   3. C++ wrapper library: middleware_rs_cpp
#
# Usage:
#   target_link_libraries(your_target PRIVATE middleware_rs_cpp)

# Import Rust crate
corrosion_import_crate(MANIFEST_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml)

# CXX bridge target: corrosion_add_cxxbridge handles include path discovery
# across all platforms/configurations
corrosion_add_cxxbridge(middleware_rs_cxxbridge
    CRATE middleware_rs
    FILES ffi/bridge.rs
)

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
        middleware_rs_cxxbridge  # CXX bridge: generated glue + include dirs for bridge.rs.h
        middleware_rs            # Rust static lib from Corrosion
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
