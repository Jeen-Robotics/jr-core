//! Rust Middleware - Zero-copy pub/sub for inter-thread communication
//!
//! This crate provides a high-performance pub/sub middleware with:
//! - Zero-copy message passing via Arc<T>
//! - Async API built on tokio
//! - C++ bindings via CXX (Phase 2)

pub mod middleware;
pub mod topic;

pub use middleware::Middleware;
