//! Core Middleware implementation

use std::sync::Arc;

use tokio::sync::broadcast;
use tracing;

use crate::topic::TopicRegistry;

/// Errors that can occur during middleware operations
#[derive(Debug, Clone)]
pub enum MiddlewareError {
    /// Topic exists with a different message type
    TypeMismatch { topic: String },
    /// Channel is lagging (missed messages)
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
}

impl<T: Send + Sync + 'static> Subscription<T> {
    /// Receive the next message
    ///
    /// Returns the message wrapped in Arc for zero-copy sharing.
    pub async fn recv(&mut self) -> Result<Arc<T>, MiddlewareError> {
        match self.receiver.recv().await {
            Ok(msg) => Ok(msg),
            Err(broadcast::error::RecvError::Lagged(n)) => {
                tracing::warn!("Subscription to '{}' lagged by {} messages", self.topic, n);
                Err(MiddlewareError::Lagged {
                    topic: self.topic.clone(),
                    count: n,
                })
            }
            Err(broadcast::error::RecvError::Closed) => {
                Err(MiddlewareError::Closed {
                    topic: self.topic.clone(),
                })
            }
        }
    }

    /// Try to receive without blocking
    ///
    /// Returns None if no message is available.
    pub fn try_recv(&mut self) -> Option<Result<Arc<T>, MiddlewareError>> {
        match self.receiver.try_recv() {
            Ok(msg) => Some(Ok(msg)),
            Err(broadcast::error::TryRecvError::Lagged(n)) => {
                tracing::warn!("Subscription to '{}' lagged by {} messages", self.topic, n);
                Some(Err(MiddlewareError::Lagged {
                    topic: self.topic.clone(),
                    count: n,
                }))
            }
            Err(broadcast::error::TryRecvError::Closed) => {
                Some(Err(MiddlewareError::Closed {
                    topic: self.topic.clone(),
                }))
            }
            Err(broadcast::error::TryRecvError::Empty) => None,
        }
    }

    /// Get the topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Resubscribe to the topic from the current position
    ///
    /// This creates a new receiver that will only see new messages.
    pub fn resubscribe(&self) -> Self {
        Self {
            receiver: self.receiver.resubscribe(),
            topic: self.topic.clone(),
        }
    }
}

/// Main middleware struct for pub/sub communication
///
/// Provides async publish/subscribe functionality with zero-copy message passing.
/// Messages are wrapped in `Arc<T>` so multiple subscribers receive the same
/// allocation without copying.
///
/// # Example
///
/// ```ignore
/// use middleware_rs::Middleware;
/// use std::sync::Arc;
///
/// #[tokio::main]
/// async fn main() {
///     let mw = Middleware::new();
///     
///     // Subscribe to a topic
///     let mut sub = mw.subscribe::<String>("greetings").await.unwrap();
///     
///     // Publish a message
///     mw.publish("greetings", Arc::new("Hello!".to_string())).await;
///     
///     // Receive the message
///     let msg = sub.recv().await.unwrap();
///     assert_eq!(*msg, "Hello!");
/// }
/// ```
pub struct Middleware {
    topics: TopicRegistry,
    runtime: tokio::runtime::Handle,
}

impl Middleware {
    /// Create a new Middleware instance
    ///
    /// Panics if called outside of a tokio runtime context.
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

    /// Subscribe to a topic
    ///
    /// Returns a Subscription handle that can be used to receive messages.
    /// The topic is created if it doesn't exist.
    ///
    /// # Type Safety
    ///
    /// Returns an error if the topic already exists with a different message type.
    pub async fn subscribe<T: Send + Sync + 'static>(
        &self,
        topic: &str,
    ) -> Result<Subscription<T>, MiddlewareError> {
        match self.topics.subscribe::<T>(topic) {
            Some(receiver) => {
                tracing::debug!("Subscribed to topic '{}'", topic);
                Ok(Subscription {
                    receiver,
                    topic: topic.to_string(),
                })
            }
            None => {
                tracing::error!("Failed to subscribe to '{}': type mismatch", topic);
                Err(MiddlewareError::TypeMismatch {
                    topic: topic.to_string(),
                })
            }
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
    pub async fn publish<T: Send + Sync + 'static>(
        &self,
        topic: &str,
        msg: Arc<T>,
    ) -> Result<usize, MiddlewareError> {
        match self.topics.publish(topic, msg) {
            Some(count) => {
                tracing::trace!("Published to '{}', {} receivers", topic, count);
                Ok(count)
            }
            None => {
                tracing::error!("Failed to publish to '{}': type mismatch", topic);
                Err(MiddlewareError::TypeMismatch {
                    topic: topic.to_string(),
                })
            }
        }
    }

    /// Publish a message, automatically wrapping it in Arc
    ///
    /// Convenience method that wraps the message in Arc before publishing.
    pub async fn publish_owned<T: Send + Sync + 'static>(
        &self,
        topic: &str,
        msg: T,
    ) -> Result<usize, MiddlewareError> {
        self.publish(topic, Arc::new(msg)).await
    }

    /// Check if a topic exists
    pub fn has_topic(&self, topic: &str) -> bool {
        self.topics.has_topic(topic)
    }

    /// Get the number of active subscribers for a topic
    pub fn subscriber_count<T: Send + Sync + 'static>(&self, topic: &str) -> Option<usize> {
        self.topics.subscriber_count::<T>(topic)
    }

    /// Remove a topic and close all its subscribers
    pub fn remove_topic(&self, topic: &str) -> bool {
        let removed = self.topics.remove_topic(topic);
        if removed {
            tracing::info!("Removed topic '{}'", topic);
        }
        removed
    }

    /// Get all topic names
    pub fn topic_names(&self) -> Vec<String> {
        self.topics.topic_names()
    }

    /// Get a reference to the runtime handle
    pub fn runtime(&self) -> &tokio::runtime::Handle {
        &self.runtime
    }
}

impl Default for Middleware {
    fn default() -> Self {
        Self::new()
    }
}

// Middleware is Send + Sync because TopicRegistry uses DashMap
unsafe impl Send for Middleware {}
unsafe impl Sync for Middleware {}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_basic_pubsub() {
        let mw = Middleware::new();

        // Subscribe first
        let mut sub = mw.subscribe::<String>("test").await.unwrap();

        // Publish a message
        let msg = Arc::new("Hello, World!".to_string());
        let count = mw.publish("test", msg.clone()).await.unwrap();
        assert_eq!(count, 1);

        // Receive the message
        let received = sub.recv().await.unwrap();
        assert_eq!(*received, "Hello, World!");
    }

    #[tokio::test]
    async fn test_multiple_subscribers_same_arc() {
        let mw = Middleware::new();

        // Create multiple subscribers
        let mut sub1 = mw.subscribe::<String>("multi").await.unwrap();
        let mut sub2 = mw.subscribe::<String>("multi").await.unwrap();
        let mut sub3 = mw.subscribe::<String>("multi").await.unwrap();

        // Publish a message
        let msg = Arc::new("shared message".to_string());
        let original_ptr = Arc::as_ptr(&msg);
        let count = mw.publish("multi", msg).await.unwrap();
        assert_eq!(count, 3);

        // All subscribers receive the same Arc (zero-copy verification)
        let r1 = sub1.recv().await.unwrap();
        let r2 = sub2.recv().await.unwrap();
        let r3 = sub3.recv().await.unwrap();

        // Verify all point to the same memory (zero-copy)
        assert_eq!(Arc::as_ptr(&r1), original_ptr);
        assert_eq!(Arc::as_ptr(&r2), original_ptr);
        assert_eq!(Arc::as_ptr(&r3), original_ptr);

        // Verify content
        assert_eq!(*r1, "shared message");
        assert_eq!(*r2, "shared message");
        assert_eq!(*r3, "shared message");
    }

    #[tokio::test]
    async fn test_unsubscribe_stops_delivery() {
        let mw = Middleware::new();

        // Subscribe
        let sub = mw.subscribe::<String>("unsub_test").await.unwrap();

        // Verify subscription is active
        assert_eq!(mw.subscriber_count::<String>("unsub_test"), Some(1));

        // Drop the subscription (unsubscribe)
        drop(sub);

        // Subscriber count should decrease
        // Note: The receiver count decreases when the Receiver is dropped
        let count = mw.subscriber_count::<String>("unsub_test").unwrap();
        assert_eq!(count, 0);

        // Publishing should still work but deliver to 0 receivers
        let result = mw
            .publish("unsub_test", Arc::new("nobody home".to_string()))
            .await
            .unwrap();
        assert_eq!(result, 0);
    }

    #[tokio::test]
    async fn test_type_mismatch() {
        let mw = Middleware::new();

        // Create topic with String type
        let _sub = mw.subscribe::<String>("typed").await.unwrap();

        // Try to publish with different type
        let result = mw.publish("typed", Arc::new(42i32)).await;
        assert!(matches!(result, Err(MiddlewareError::TypeMismatch { .. })));

        // Try to subscribe with different type
        let result = mw.subscribe::<i32>("typed").await;
        assert!(matches!(result, Err(MiddlewareError::TypeMismatch { .. })));
    }

    #[tokio::test]
    async fn test_publish_owned() {
        let mw = Middleware::new();

        let mut sub = mw.subscribe::<String>("owned").await.unwrap();

        // Use publish_owned which auto-wraps in Arc
        mw.publish_owned("owned", "auto-wrapped".to_string())
            .await
            .unwrap();

        let msg = sub.recv().await.unwrap();
        assert_eq!(*msg, "auto-wrapped");
    }

    #[tokio::test]
    async fn test_multiple_messages() {
        let mw = Middleware::new();

        let mut sub = mw.subscribe::<i32>("numbers").await.unwrap();

        // Publish multiple messages
        for i in 0..5 {
            mw.publish_owned("numbers", i).await.unwrap();
        }

        // Receive all in order
        for i in 0..5 {
            let msg = sub.recv().await.unwrap();
            assert_eq!(*msg, i);
        }
    }

    #[tokio::test]
    async fn test_topic_names() {
        let mw = Middleware::new();

        mw.subscribe::<String>("topic1").await.unwrap();
        mw.subscribe::<i32>("topic2").await.unwrap();
        mw.subscribe::<Vec<u8>>("topic3").await.unwrap();

        let names = mw.topic_names();
        assert_eq!(names.len(), 3);
        assert!(names.contains(&"topic1".to_string()));
        assert!(names.contains(&"topic2".to_string()));
        assert!(names.contains(&"topic3".to_string()));
    }

    #[tokio::test]
    async fn test_remove_topic() {
        let mw = Middleware::new();

        let _sub = mw.subscribe::<String>("removable").await.unwrap();
        assert!(mw.has_topic("removable"));

        mw.remove_topic("removable");
        assert!(!mw.has_topic("removable"));
    }

    #[tokio::test]
    async fn test_resubscribe() {
        let mw = Middleware::new();

        let sub1 = mw.subscribe::<String>("resub").await.unwrap();

        // Publish a message that sub1 will miss (it's not awaiting yet)
        mw.publish_owned("resub", "first".to_string())
            .await
            .unwrap();

        // Create a new subscription from sub1 - starts fresh
        let mut sub2 = sub1.resubscribe();

        // Publish another message
        mw.publish_owned("resub", "second".to_string())
            .await
            .unwrap();

        // sub2 should receive "second" but not "first"
        let msg = sub2.recv().await.unwrap();
        assert_eq!(*msg, "second");
    }

    #[tokio::test]
    async fn test_try_recv() {
        let mw = Middleware::new();

        let mut sub = mw.subscribe::<String>("try_recv").await.unwrap();

        // No message yet
        assert!(sub.try_recv().is_none());

        // Publish
        mw.publish_owned("try_recv", "available".to_string())
            .await
            .unwrap();

        // Now there's a message
        let result = sub.try_recv();
        assert!(result.is_some());
        assert_eq!(*result.unwrap().unwrap(), "available");

        // Empty again
        assert!(sub.try_recv().is_none());
    }
}
