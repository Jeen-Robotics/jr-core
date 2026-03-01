//! Topic registry for managing pub/sub channels

use std::sync::Arc;
use std::any::Any;
use dashmap::DashMap;
use tokio::sync::broadcast;

/// Registry for all topics and their channels
pub struct TopicRegistry {
    topics: DashMap<String, TopicChannel>,
}

struct TopicChannel {
    sender: broadcast::Sender<Arc<dyn Any + Send + Sync>>,
}

impl TopicRegistry {
    pub fn new() -> Self {
        Self {
            topics: DashMap::new(),
        }
    }

    // TODO: Implement get_or_create, publish, subscribe
}

impl Default for TopicRegistry {
    fn default() -> Self {
        Self::new()
    }
}
