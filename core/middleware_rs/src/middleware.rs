//! Core Middleware implementation

use std::sync::Arc;
use dashmap::DashMap;
use tokio::sync::broadcast;

use crate::topic::TopicRegistry;

/// Main middleware struct for pub/sub communication
pub struct Middleware {
    topics: TopicRegistry,
    runtime: tokio::runtime::Handle,
}

impl Middleware {
    /// Create a new Middleware instance
    pub fn new() -> Self {
        Self {
            topics: TopicRegistry::new(),
            runtime: tokio::runtime::Handle::current(),
        }
    }

    /// Create Middleware with a specific runtime handle
    pub fn with_runtime(handle: tokio::runtime::Handle) -> Self {
        Self {
            topics: TopicRegistry::new(),
            runtime: handle,
        }
    }

    // TODO: Implement publish/subscribe in Phase 1
}

impl Default for Middleware {
    fn default() -> Self {
        Self::new()
    }
}
