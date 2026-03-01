//! FFI module for C++ interop via CXX
//!
//! Provides a C++ interface to the Rust middleware, enabling:
//! - Publishing messages from C++ to Rust subscribers
//! - Subscribing from C++ to Rust publishers
//! - Messages passed as serialized bytes for cross-language compatibility
//!
//! # Thread Safety
//!
//! All FFI functions are thread-safe. The global middleware instance
//! is protected by internal synchronization.

mod bridge;

pub use bridge::ffi::*;
