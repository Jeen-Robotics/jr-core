//! Rust Middleware - Zero-copy pub/sub for inter-thread communication
//!
//! This crate provides a high-performance pub/sub middleware with:
//! - Zero-copy message passing via Arc<T>
//! - Async API built on tokio
//! - Type-safe topics with runtime type checking
//!
//! # Quick Start
//!
//! ```ignore
//! use middleware::Middleware;
//! use std::sync::Arc;
//!
//! #[tokio::main]
//! async fn main() {
//!     let mw = Middleware::new();
//!     
//!     // Subscribe to a topic
//!     let mut sub = mw.subscribe::<String>("greetings").unwrap();
//!     
//!     // Publish a message
//!     mw.publish("greetings", Arc::new("Hello!".to_string())).unwrap();
//!     
//!     // Receive the message (zero-copy - same Arc, async recv)
//!     let msg = sub.recv().await.unwrap();
//!     println!("Received: {}", *msg);
//! }
//! ```
//!
//! # Zero-Copy Design
//!
//! Messages are wrapped in `Arc<T>` and broadcast to all subscribers.
//! Each subscriber receives a clone of the Arc (just incrementing the
//! reference count), not a copy of the data. This is true zero-copy
//! for large messages.
//!
//! # Type Safety
//!
//! Topics are dynamically typed but type-checked at runtime. Once a topic
//! is created with type T, all publish/subscribe operations must use the
//! same type T. Type mismatches return errors.

pub mod middleware;
pub mod topic;
pub mod ffi;

pub use middleware::{Middleware, MiddlewareError, Subscription};
pub use topic::{Qos, TopicRegistry};
