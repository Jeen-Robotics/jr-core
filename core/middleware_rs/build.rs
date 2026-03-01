//! Build script for middleware_rs
//!
//! Generates Rust types from .proto files using prost-build.
//! All proto definitions come from jr-msgs submodule (shared with C++).
//! CXX bridge generation will be added in Phase 2.

use std::path::PathBuf;

fn main() {
    // Use CARGO_MANIFEST_DIR for reliable path resolution
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let jr_msgs_path = manifest_dir.join("../../external/jr-msgs");

    // Proto files from jr-msgs
    let proto_files = [
        jr_msgs_path.join("std_msgs.proto"),
        jr_msgs_path.join("geometry_msgs.proto"),
        jr_msgs_path.join("sensor_msgs.proto"),
        jr_msgs_path.join("bag.proto"),
    ];

    // Rerun if any proto file changes (not directory - Cargo doesn't watch recursively)
    for proto in &proto_files {
        println!("cargo:rerun-if-changed={}", proto.display());
    }
    println!("cargo:rerun-if-changed=build.rs");

    // Compile protos (out_dir defaults to OUT_DIR, no need to set explicitly)
    prost_build::Config::new()
        .compile_protos(&proto_files, &[&jr_msgs_path])
        .expect("Failed to compile proto files");
}
