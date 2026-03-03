//! Core Middleware implementation

use std::sync::Arc;

use tokio::sync::broadcast;

use crate::topic::{Qos, TopicRegistry};

/// Errors that can occur during middleware operations
#[derive(Debug, Clone)]
pub enum MiddlewareError {
    /// Topic exists with a different message type
    TypeMismatch { topic: String },
    /// Channel is lagging (missed messages) - only for KeepLast QoS
    Lagged { topic: String, count: u64 },
    /// Channel is closed
    Closed { topic: String },
}

impl std::fmt::Display for MiddlewareError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MiddlewareError::TypeMismatch { topic } => {
                write!(f, "Type mismatch for topic '{}'", topic)
            }
            MiddlewareError::Lagged { topic, count } => {
                write!(f, "Lagged {} messages on topic '{}'", count, topic)
            }
            MiddlewareError::Closed { topic } => {
                write!(f, "Channel closed for topic '{}'", topic)
            }
        }
    }
}

impl std::error::Error for MiddlewareError {}

/// A subscription handle that allows receiving messages from a topic
pub struct Subscription<T: Send + Sync + 'static> {
    receiver: broadcast::Receiver<Arc<T>>,
    topic: String,
    qos: Qos,
}

impl<T: Send + Sync + 'static> Subscription<T> {
    /// Receive the next message
    ///
    /// For `Qos::SensorData`, lagged messages are silently skipped and the
    /// latest available message is returned. For `Qos::KeepLast`, lagging
    /// returns an error.
    ///
    /// Returns the message wrapped in Arc for zero-copy sharing.
    pub async fn recv(&mut self) -> Result<Arc<T>, MiddlewareError> {
        loop {
            match self.receiver.recv().await {
                Ok(msg) => return Ok(msg),
                Err(broadcast::error::RecvError::Lagged(n)) => {
                    if self.qos.skip_lagged() {
                        // SensorData QoS: silently skip to latest, retry
                        continue;
                    } else {
                        eprintln!(
                            "[middleware] WARN: Subscription to '{}' lagged by {} messages",
                            self.topic, n
                        );
                        return Err(MiddlewareError::Lagged {
                            topic: self.topic.clone(),
                            count: n,
                        });
                    }
                }
                Err(broadcast::error::RecvError::Closed) => {
                    return Err(MiddlewareError::Closed {
                        topic: self.topic.clone(),
                    });
                }
            }
        }
    }

    /// Try to receive without blocking
    ///
    /// For `Qos::SensorData`, lagged messages are silently skipped.
    /// Returns None if no message is available.
    pub fn try_recv(&mut self) -> Option<Result<Arc<T>, MiddlewareError>> {
        loop {
            match self.receiver.try_recv() {
                Ok(msg) => return Some(Ok(msg)),
                Err(broadcast::error::TryRecvError::Lagged(n)) => {
                    if self.qos.skip_lagged() {
                        // SensorData QoS: silently skip, retry
                        continue;
                    } else {
                        eprintln!(
                            "[middleware] WARN: Subscription to '{}' lagged by {} messages",
                            self.topic, n
                        );
                        return Some(Err(MiddlewareError::Lagged {
                            topic: self.topic.clone(),
                            count: n,
                        }));
                    }
                }
                Err(broadcast::error::TryRecvError::Closed) => {
                    return Some(Err(MiddlewareError::Closed {
                        topic: self.topic.clone(),
                    }));
                }
                Err(broadcast::error::TryRecvError::Empty) => return None,
            }
        }
    }

    /// Get the topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Get the QoS configuration
    pub fn qos(&self) -> Qos {
        self.qos
    }

    /// Creates a new receiver that sees only messages published *after* this call.
    ///
    /// Buffered messages in the current receiver are not replicated to the new one.
    /// This is useful when you want to "catch up" by skipping old messages.
    pub fn resubscribe(&self) -> Self {
        Self {
            receiver: self.receiver.resubscribe(),
            topic: self.topic.clone(),
            qos: self.qos,
        }
    }
}

/// Main middleware struct for pub/sub communication
///
/// Provides publish/subscribe functionality with zero-copy message passing.
/// Messages are wrapped in `Arc<T>` so multiple subscribers receive the same
/// allocation without copying.
///
/// # Example
///
/// ```ignore
/// use middleware::{Middleware, Qos};
/// use std::sync::Arc;
///
/// let mw = Middleware::new();
/// 
/// // Subscribe to a topic (default QoS)
/// let mut sub = mw.subscribe::<String>("greetings").unwrap();
/// 
/// // Subscribe to sensor data (realtime, latest-only)
/// let mut imu = mw.subscribe_with_qos::<f64>("imu", Qos::SensorData).unwrap();
/// 
/// // Publish a message
/// mw.publish("greetings", Arc::new("Hello!".to_string()));
/// 
/// // Receive the message (async)
/// let msg = sub.recv().await.unwrap();
/// assert_eq!(*msg, "Hello!");
/// ```
pub struct Middleware {
    topics: TopicRegistry,
}

impl Middleware {
    /// Create a new Middleware instance
    pub fn new() -> Self {
        Self {
            topics: TopicRegistry::new(),
        }
    }

    /// Subscribe to a topic with default QoS (KeepLast(256))
    ///
    /// Returns a Subscription handle that can be used to receive messages.
    /// The topic is created if it doesn't exist.
    ///
    /// # Type Safety
    ///
    /// Returns an error if the topic already exists with a different message type.
    pub fn subscribe<T: Send + Sync + 'static>(
        &self,
        topic: &str,
    ) -> Result<Subscription<T>, MiddlewareError> {
        self.subscribe_with_qos(topic, Qos::default())
    }

    /// Subscribe to a topic with specified QoS
    ///
    /// # QoS Options
    ///
    /// - `Qos::KeepLast(n)` - Buffer n messages, return Lagged error if slow
    /// - `Qos::SensorData` - Always get latest, silently skip old messages
    ///
    /// # Example
    ///
    /// ```ignore
    /// // For IMU data at 400Hz, we only care about the latest reading
    /// let mut imu = mw.subscribe_with_qos::<ImuData>("imu", Qos::SensorData)?;
    /// 
    /// // For logs, we want reliable delivery with a large buffer
    /// let mut logs = mw.subscribe_with_qos::<String>("logs", Qos::KeepLast(1024))?;
    /// ```
    pub fn subscribe_with_qos<T: Send + Sync + 'static>(
        &self,
        topic: &str,
        qos: Qos,
    ) -> Result<Subscription<T>, MiddlewareError> {
        match self.topics.subscribe_with_qos::<T>(topic, qos) {
            Some(receiver) => Ok(Subscription {
                receiver,
                topic: topic.to_string(),
                qos,
            }),
            None => Err(MiddlewareError::TypeMismatch {
                topic: topic.to_string(),
            }),
        }
    }

    /// Publish a message to a topic
    ///
    /// The message is wrapped in Arc for zero-copy delivery to all subscribers.
    /// Returns the number of subscribers that received the message.
    ///
    /// # Type Safety
    ///
    /// Returns an error if the topic already exists with a different message type.
    pub fn publish<T: Send + Sync + 'static>(
        &self,
        topic: &str,
        msg: Arc<T>,
    ) -> Result<usize, MiddlewareError> {
        match self.topics.publish(topic, msg) {
            Some(count) => Ok(count),
            None => Err(MiddlewareError::TypeMismatch {
                topic: topic.to_string(),
            }),
        }
    }

    /// Publish a message, automatically wrapping it in Arc
    ///
    /// Convenience method that wraps the message in Arc before publishing.
    pub fn publish_owned<T: Send + Sync + 'static>(
        &self,
        topic: &str,
        msg: T,
    ) -> Result<usize, MiddlewareError> {
        self.publish(topic, Arc::new(msg))
    }

    /// Check if a topic exists
    pub fn has_topic(&self, topic: &str) -> bool {
        self.topics.has_topic(topic)
    }

    /// Get the QoS configuration for a topic
    pub fn get_qos(&self, topic: &str) -> Option<Qos> {
        self.topics.get_qos(topic)
    }

    /// Get the number of active subscribers for a topic
    pub fn subscriber_count<T: Send + Sync + 'static>(&self, topic: &str) -> Option<usize> {
        self.topics.subscriber_count::<T>(topic)
    }

    /// Remove a topic and close all its subscribers
    pub fn remove_topic(&self, topic: &str) -> bool {
        self.topics.remove_topic(topic)
    }

    /// Get all topic names
    pub fn topic_names(&self) -> Vec<String> {
        self.topics.topic_names()
    }

    /// Subscribe to a topic with raw bytes type (for FFI)
    ///
    /// Returns the underlying broadcast receiver for polling-based FFI use.
    pub fn subscribe_raw(
        &self,
        topic: &str,
        qos: Qos,
    ) -> Option<tokio::sync::broadcast::Receiver<std::sync::Arc<Vec<u8>>>> {
        self.topics.subscribe_with_qos::<Vec<u8>>(topic, qos)
    }

    /// Get or create a sender for raw bytes type (for FFI)
    ///
    /// Returns the underlying broadcast sender for cached publishing.
    pub fn get_or_create_sender_raw(
        &self,
        topic: &str,
        qos: Qos,
    ) -> Option<tokio::sync::broadcast::Sender<std::sync::Arc<Vec<u8>>>> {
        self.topics.get_or_create_sender_with_qos::<Vec<u8>>(topic, qos)
    }
}

impl Default for Middleware {
    fn default() -> Self {
        Self::new()
    }
}

// Note: Middleware is automatically Send + Sync because TopicRegistry
// (via DashMap) implements Send + Sync
