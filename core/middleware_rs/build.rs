//! Build script for middleware_rs
//!
//! Generates:
//! - C++ headers and sources from CXX bridge definitions

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/ffi/bridge.rs");

    // Compile CXX bridge
    cxx_build::bridge("src/ffi/bridge.rs")
        .std("c++17")
        .compile("middleware_rs_cxx");
}
