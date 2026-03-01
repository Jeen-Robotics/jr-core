//! Build script for middleware_rs
//!
//! Generates Rust types from .proto files using prost-build.
//! All proto definitions come from jr-msgs submodule (shared with C++).
//! CXX bridge generation will be added in Phase 2.

use std::path::PathBuf;

fn main() {
    // Path to jr-msgs submodule
    let jr_msgs_path = PathBuf::from("../../external/jr-msgs");
    
    // Rerun if proto files change
    println!("cargo:rerun-if-changed={}", jr_msgs_path.display());
    println!("cargo:rerun-if-changed=build.rs");

    // Proto files from jr-msgs
    let proto_files = [
        jr_msgs_path.join("std_msgs.proto"),
        jr_msgs_path.join("geometry_msgs.proto"),
        jr_msgs_path.join("sensor_msgs.proto"),
        jr_msgs_path.join("bag.proto"),
    ];
    
    prost_build::Config::new()
        .out_dir(std::env::var("OUT_DIR").unwrap())
        .compile_protos(&proto_files, &[&jr_msgs_path])
        .expect("Failed to compile proto files");
}
