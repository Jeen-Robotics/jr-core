# Rust middleware with C++ wrapper
#
# Builds:
#   1. Rust static library via Corrosion
#   2. C++ wrapper that bridges to CXX-generated FFI
#   3. Combined library target: middleware_rs_cpp
#
# Usage:
#   target_link_libraries(your_target PRIVATE middleware_rs_cpp)

# Import Rust crate
corrosion_import_crate(MANIFEST_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml)

# CXX bridge headers are generated during cargo build
# When using Corrosion, headers are in: ${CMAKE_BINARY_DIR}/cargo/build/.../cxxbridge/
# We need to use corrosion's exported include directories
# For now, use the Corrosion target directory
set(CORROSION_TARGET_DIR "${CMAKE_BINARY_DIR}/cargo/build")
# CXX outputs headers to target/cxxbridge/<crate>/src/... during build
# We also check the source target dir for local builds
set(CXX_BRIDGE_DIR_BUILD "${CORROSION_TARGET_DIR}/cxxbridge")
set(CXX_BRIDGE_DIR_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/target/cxxbridge")

# Find protobuf for the wrapper
find_package(Protobuf REQUIRED)

# C++ wrapper library
add_library(middleware_rs_cpp STATIC
    src/wrapper.cpp
)

target_include_directories(middleware_rs_cpp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        # CXX-generated headers (need to build Rust first)
        # Both build dir (Corrosion) and source dir (local cargo build) for compatibility
        ${CXX_BRIDGE_DIR_BUILD}
        ${CXX_BRIDGE_DIR_SOURCE}
)

# Link against Rust library and dependencies
target_link_libraries(middleware_rs_cpp
    PUBLIC
        protobuf::libprotobuf
    PRIVATE
        middleware_rs  # Rust static lib from Corrosion
)

# Ensure Rust builds before C++ wrapper
add_dependencies(middleware_rs_cpp cargo-build_middleware_rs)

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
