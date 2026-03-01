//! Build script for middleware_rs
//!
//! Generates Rust types from .proto files using prost-build.
//! - Common messages (std_msgs, sensor_msgs, etc.) come from jr-msgs submodule
//! - Middleware-specific messages (bag.proto) are kept locally
//! CXX bridge generation will be added in Phase 2.

use std::path::PathBuf;

fn main() {
    // Paths
    let jr_msgs_path = PathBuf::from("../../external/jr-msgs");
    let local_proto_path = PathBuf::from("proto");
    
    // Rerun if proto files change
    println!("cargo:rerun-if-changed={}", jr_msgs_path.display());
    println!("cargo:rerun-if-changed={}", local_proto_path.display());
    println!("cargo:rerun-if-changed=build.rs");

    // Proto files from jr-msgs (shared with C++)
    let jr_msgs_protos = [
        jr_msgs_path.join("std_msgs.proto"),
        jr_msgs_path.join("geometry_msgs.proto"),
        jr_msgs_path.join("sensor_msgs.proto"),
    ];
    
    // Middleware-specific proto files
    let local_protos = [
        local_proto_path.join("bag.proto"),
    ];
    
    // Combine all proto files
    let all_protos: Vec<PathBuf> = jr_msgs_protos
        .iter()
        .chain(local_protos.iter())
        .cloned()
        .collect();
    
    // Include paths for proto imports
    let include_paths = [
        jr_msgs_path.as_path(),
        local_proto_path.as_path(),
    ];
    
    prost_build::Config::new()
        .out_dir(std::env::var("OUT_DIR").unwrap())
        .compile_protos(&all_protos, &include_paths)
        .expect("Failed to compile proto files");
}
