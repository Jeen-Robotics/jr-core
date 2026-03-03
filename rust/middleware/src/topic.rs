//! Topic registry for managing pub/sub channels

use std::any::{Any, TypeId};
use std::sync::Arc;

use dashmap::DashMap;
use tokio::sync::broadcast;

/// Quality of Service configuration for topics
///
/// Controls buffering behavior and how slow consumers are handled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Qos {
    /// Default behavior: buffer up to `capacity` messages.
    /// Slow consumers receive `Lagged` error when they fall behind.
    /// Good for: reliable message delivery where order matters.
    KeepLast(usize),

    /// Realtime/sensor behavior: always get the latest message.
    /// Equivalent to `KeepLast(1)` but `recv()` silently skips to newest
    /// instead of returning `Lagged` error.
    /// Good for: IMU, odometry, sensor readings where only latest matters.
    SensorData,
}

impl Default for Qos {
    fn default() -> Self {
        Qos::KeepLast(256)
    }
}

impl Qos {
    /// Get the channel capacity for this QoS setting
    pub fn capacity(&self) -> usize {
        match self {
            Qos::KeepLast(n) => *n,
            Qos::SensorData => 1,
        }
    }

    /// Whether to silently skip lagged messages
    pub fn skip_lagged(&self) -> bool {
        matches!(self, Qos::SensorData)
    }
}

/// Type-erased topic channel that stores broadcast sender
struct TopicChannel {
    /// Type-erased sender (actually broadcast::Sender<Arc<T>>)
    sender: Box<dyn Any + Send + Sync>,
    /// TypeId of the message type for runtime type checking
    type_id: TypeId,
    /// QoS configuration for this topic
    qos: Qos,
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

    /// Get or create a broadcast sender for a topic with specified QoS
    ///
    /// If the topic doesn't exist, creates a new broadcast channel with given QoS.
    /// If it exists with a different type, returns None (type mismatch).
    /// If it exists with matching type, returns the existing sender (QoS is ignored).
    pub fn get_or_create_sender_with_qos<T: Send + Sync + 'static>(
        &self,
        topic: &str,
        qos: Qos,
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

        // Create new channel with QoS capacity
        let (sender, _) = broadcast::channel::<Arc<T>>(qos.capacity());
        let sender_clone = sender.clone();

        let channel = TopicChannel {
            sender: Box::new(sender),
            type_id,
            qos,
        };

        // Use entry API to handle concurrent inserts
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
                entry.insert(channel);
                Some(sender_clone)
            }
        }
    }

    /// Get or create a broadcast sender with default QoS
    pub fn get_or_create_sender<T: Send + Sync + 'static>(
        &self,
        topic: &str,
    ) -> Option<broadcast::Sender<Arc<T>>> {
        self.get_or_create_sender_with_qos(topic, Qos::default())
    }

    /// Subscribe to a topic with specified QoS, returns a receiver
    ///
    /// If the topic doesn't exist, creates it with given QoS.
    /// If the topic exists with a different type, returns None.
    pub fn subscribe_with_qos<T: Send + Sync + 'static>(
        &self,
        topic: &str,
        qos: Qos,
    ) -> Option<broadcast::Receiver<Arc<T>>> {
        self.get_or_create_sender_with_qos::<T>(topic, qos)
            .map(|sender| sender.subscribe())
    }

    /// Subscribe to a topic with default QoS
    pub fn subscribe<T: Send + Sync + 'static>(
        &self,
        topic: &str,
    ) -> Option<broadcast::Receiver<Arc<T>>> {
        self.subscribe_with_qos(topic, Qos::default())
    }

    /// Get the QoS configuration for a topic
    pub fn get_qos(&self, topic: &str) -> Option<Qos> {
        self.topics.get(topic).map(|e| e.qos)
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
