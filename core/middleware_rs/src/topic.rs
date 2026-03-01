//! Topic registry for managing pub/sub channels

use std::any::{Any, TypeId};
use std::sync::Arc;

use dashmap::DashMap;
use tokio::sync::broadcast;

/// Default channel capacity for broadcast channels
const DEFAULT_CHANNEL_CAPACITY: usize = 256;

/// Type-erased topic channel that stores broadcast sender
struct TopicChannel {
    /// Type-erased sender (actually broadcast::Sender<Arc<T>>)
    sender: Box<dyn Any + Send + Sync>,
    /// TypeId of the message type for runtime type checking
    type_id: TypeId,
}

/// Registry for all topics and their channels
///
/// Each topic is identified by a string name and holds a typed broadcast channel.
/// Multiple subscribers can receive the same Arc<T> (zero-copy).
pub struct TopicRegistry {
    /// Map from topic name to type-erased channel
    topics: DashMap<String, TopicChannel>,
}

impl TopicRegistry {
    /// Create a new empty TopicRegistry
    pub fn new() -> Self {
        Self {
            topics: DashMap::new(),
        }
    }

    /// Get or create a broadcast sender for a topic
    ///
    /// If the topic doesn't exist, creates a new broadcast channel.
    /// If it exists with a different type, returns None (type mismatch).
    pub fn get_or_create_sender<T: Send + Sync + 'static>(
        &self,
        topic: &str,
    ) -> Option<broadcast::Sender<Arc<T>>> {
        let type_id = TypeId::of::<T>();

        // Check if topic exists
        if let Some(entry) = self.topics.get(topic) {
            // Verify type matches
            if entry.type_id != type_id {
                return None; // Type mismatch
            }
            // Downcast and clone the sender
            let sender = entry
                .sender
                .downcast_ref::<broadcast::Sender<Arc<T>>>()
                .expect("Type ID matched but downcast failed - this is a bug");
            return Some(sender.clone());
        }

        // Create new channel
        let (sender, _) = broadcast::channel::<Arc<T>>(DEFAULT_CHANNEL_CAPACITY);
        let sender_clone = sender.clone();

        // Try to insert, handling race conditions
        let channel = TopicChannel {
            sender: Box::new(sender),
            type_id,
        };

        // Use entry API to handle concurrent inserts
        // or_insert_with would be better but we need to handle the case
        // where another thread inserted a different type
        match self.topics.entry(topic.to_string()) {
            dashmap::mapref::entry::Entry::Occupied(entry) => {
                // Another thread inserted first, verify type
                if entry.get().type_id != type_id {
                    return None;
                }
                entry
                    .get()
                    .sender
                    .downcast_ref::<broadcast::Sender<Arc<T>>>()
                    .map(|s| s.clone())
            }
            dashmap::mapref::entry::Entry::Vacant(entry) => {
                // We're first, insert and return our sender
                entry.insert(channel);
                Some(sender_clone)
            }
        }
    }

    /// Subscribe to a topic, returns a receiver
    ///
    /// If the topic doesn't exist, creates it.
    /// If the topic exists with a different type, returns None.
    pub fn subscribe<T: Send + Sync + 'static>(
        &self,
        topic: &str,
    ) -> Option<broadcast::Receiver<Arc<T>>> {
        self.get_or_create_sender::<T>(topic)
            .map(|sender| sender.subscribe())
    }

    /// Publish a message to a topic
    ///
    /// Returns the number of receivers that received the message.
    /// Returns None if the topic exists with a different type.
    pub fn publish<T: Send + Sync + 'static>(
        &self,
        topic: &str,
        msg: Arc<T>,
    ) -> Option<usize> {
        let sender = self.get_or_create_sender::<T>(topic)?;
        match sender.send(msg) {
            Ok(n) => Some(n),
            Err(_) => Some(0), // No active receivers
        }
    }

    /// Check if a topic exists
    pub fn has_topic(&self, topic: &str) -> bool {
        self.topics.contains_key(topic)
    }

    /// Get the number of active subscribers for a topic
    ///
    /// Returns None if topic doesn't exist or type doesn't match.
    /// Does NOT create the topic if it doesn't exist.
    pub fn subscriber_count<T: Send + Sync + 'static>(&self, topic: &str) -> Option<usize> {
        let entry = self.topics.get(topic)?;
        if entry.type_id != TypeId::of::<T>() {
            return None;
        }
        entry
            .sender
            .downcast_ref::<broadcast::Sender<Arc<T>>>()
            .map(|s| s.receiver_count())
    }

    /// Remove a topic from the registry
    ///
    /// Returns true if the topic existed and was removed.
    pub fn remove_topic(&self, topic: &str) -> bool {
        self.topics.remove(topic).is_some()
    }

    /// Get all topic names
    pub fn topic_names(&self) -> Vec<String> {
        self.topics.iter().map(|e| e.key().clone()).collect()
    }
}

impl Default for TopicRegistry {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_topic() {
        let registry = TopicRegistry::new();
        let sender = registry.get_or_create_sender::<String>("test");
        assert!(sender.is_some());
        assert!(registry.has_topic("test"));
    }

    #[test]
    fn test_type_mismatch() {
        let registry = TopicRegistry::new();
        
        // Create topic with String type
        let _ = registry.get_or_create_sender::<String>("test");
        
        // Try to get with different type - should fail
        let result = registry.get_or_create_sender::<i32>("test");
        assert!(result.is_none());
    }

    #[test]
    fn test_subscribe_creates_topic() {
        let registry = TopicRegistry::new();
        let rx = registry.subscribe::<String>("new_topic");
        assert!(rx.is_some());
        assert!(registry.has_topic("new_topic"));
    }

    #[test]
    fn test_publish_no_subscribers() {
        let registry = TopicRegistry::new();
        let count = registry.publish("empty", Arc::new("hello".to_string()));
        assert_eq!(count, Some(0));
    }

    #[test]
    fn test_remove_topic() {
        let registry = TopicRegistry::new();
        registry.get_or_create_sender::<String>("removable");
        assert!(registry.has_topic("removable"));
        assert!(registry.remove_topic("removable"));
        assert!(!registry.has_topic("removable"));
    }
}
