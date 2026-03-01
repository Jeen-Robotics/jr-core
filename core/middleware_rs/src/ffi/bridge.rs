//! CXX bridge definitions for C++ interop
//!
//! Uses a polling API for subscribers instead of callbacks (CXX limitation).
//! C++ code calls `subscriber_try_recv()` to poll for messages.

use std::sync::Arc;

use once_cell::sync::OnceCell;
use tokio::runtime::Runtime;
use tokio::sync::broadcast;

use crate::topic::Qos;
use crate::Middleware;

/// Global middleware instance
static MIDDLEWARE: OnceCell<Arc<MiddlewareHandle>> = OnceCell::new();

/// Handle that owns the middleware and tokio runtime
struct MiddlewareHandle {
    middleware: Middleware,
    #[allow(dead_code)]
    runtime: Runtime,
}

/// Get or initialize the global middleware
fn get_middleware() -> &'static Arc<MiddlewareHandle> {
    MIDDLEWARE.get_or_init(|| {
        let runtime = Runtime::new().expect("Failed to create tokio runtime");
        Arc::new(MiddlewareHandle {
            middleware: Middleware::new(),
            runtime,
        })
    })
}

/// Publisher handle for C++
pub struct RustPublisher {
    topic: String,
    #[allow(dead_code)]
    qos: Qos,
}

/// Subscriber handle for C++ (polling-based)
pub struct RustSubscriber {
    topic: String,
    receiver: broadcast::Receiver<Arc<Vec<u8>>>,
    qos: Qos,
}

#[cxx::bridge(namespace = "jr::mw")]
pub mod ffi {
    /// QoS configuration for topics
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum QosKind {
        /// Buffer n messages, return error if slow (reliable)
        KeepLast,
        /// Always get latest, silently skip old (realtime)
        SensorData,
    }

    /// Result of a publish operation
    #[derive(Debug)]
    pub struct PublishResult {
        pub success: bool,
        pub receivers: usize,
        pub error_msg: String,
    }

    /// Result of a receive operation
    #[derive(Debug)]
    pub struct RecvResult {
        /// True if a message was received
        pub has_message: bool,
        /// The message data (empty if no message)
        pub data: Vec<u8>,
        /// True if the channel is closed
        pub closed: bool,
        /// Number of messages that were skipped (lagged)
        pub lagged: u64,
    }

    extern "Rust" {
        // Opaque Rust types
        type RustPublisher;
        type RustSubscriber;

        // Middleware lifecycle
        fn middleware_init() -> bool;
        fn middleware_shutdown();

        // Publisher API
        fn create_publisher(topic: &str, qos: QosKind, capacity: usize) -> Box<RustPublisher>;
        fn publish_bytes(publisher: &RustPublisher, data: &[u8]) -> PublishResult;

        // Subscriber API (polling-based)
        fn create_subscriber(topic: &str, qos: QosKind, capacity: usize)
            -> Box<RustSubscriber>;
        fn subscriber_try_recv(subscriber: &mut RustSubscriber) -> RecvResult;
        fn subscriber_topic(subscriber: &RustSubscriber) -> String;

        // Utilities
        fn topic_exists(topic: &str) -> bool;
        fn list_topics() -> Vec<String>;
        fn subscriber_count(topic: &str) -> usize;
    }
}

use ffi::{PublishResult, QosKind, RecvResult};

/// Convert FFI QosKind to internal Qos
fn qos_from_ffi(kind: QosKind, capacity: usize) -> Qos {
    match kind {
        QosKind::KeepLast => Qos::KeepLast(capacity),
        QosKind::SensorData => Qos::SensorData,
        _ => Qos::default(),
    }
}

/// Initialize the middleware (idempotent)
fn middleware_init() -> bool {
    get_middleware();
    true
}

/// Shutdown the middleware
///
/// Note: Due to static lifetime, actual cleanup happens at process exit.
/// This function is provided for explicit lifecycle management.
fn middleware_shutdown() {
    // OnceCell doesn't support reset, so this is a no-op for now.
    // The middleware will be cleaned up when the process exits.
}

/// Create a publisher for a topic
fn create_publisher(topic: &str, qos: QosKind, capacity: usize) -> Box<RustPublisher> {
    let qos = qos_from_ffi(qos, capacity);
    
    // Ensure topic exists
    let handle = get_middleware();
    let _ = handle.middleware.subscribe_with_qos::<Vec<u8>>(topic, qos);
    
    Box::new(RustPublisher {
        topic: topic.to_string(),
        qos,
    })
}

/// Publish serialized bytes to a topic
fn publish_bytes(publisher: &RustPublisher, data: &[u8]) -> PublishResult {
    let handle = get_middleware();

    // Create a Vec from the data (this is a copy, unavoidable for FFI)
    let msg = Arc::new(data.to_vec());

    match handle.middleware.publish(&publisher.topic, msg) {
        Ok(receivers) => PublishResult {
            success: true,
            receivers,
            error_msg: String::new(),
        },
        Err(e) => PublishResult {
            success: false,
            receivers: 0,
            error_msg: e.to_string(),
        },
    }
}

/// Create a subscriber for a topic (polling-based)
fn create_subscriber(topic: &str, qos: QosKind, capacity: usize) -> Box<RustSubscriber> {
    let handle = get_middleware();
    let qos = qos_from_ffi(qos, capacity);

    // Get the underlying broadcast receiver directly from TopicRegistry
    let receiver = handle
        .middleware
        .topics()
        .subscribe_with_qos::<Vec<u8>>(topic, qos)
        .expect("Failed to create subscriber");

    Box::new(RustSubscriber {
        topic: topic.to_string(),
        receiver,
        qos,
    })
}

/// Try to receive a message without blocking
fn subscriber_try_recv(subscriber: &mut RustSubscriber) -> RecvResult {
    loop {
        match subscriber.receiver.try_recv() {
            Ok(msg) => {
                return RecvResult {
                    has_message: true,
                    data: (*msg).clone(),
                    closed: false,
                    lagged: 0,
                };
            }
            Err(broadcast::error::TryRecvError::Empty) => {
                return RecvResult {
                    has_message: false,
                    data: Vec::new(),
                    closed: false,
                    lagged: 0,
                };
            }
            Err(broadcast::error::TryRecvError::Lagged(n)) => {
                if subscriber.qos.skip_lagged() {
                    // SensorData: silently retry to get latest
                    continue;
                }
                return RecvResult {
                    has_message: false,
                    data: Vec::new(),
                    closed: false,
                    lagged: n,
                };
            }
            Err(broadcast::error::TryRecvError::Closed) => {
                return RecvResult {
                    has_message: false,
                    data: Vec::new(),
                    closed: true,
                    lagged: 0,
                };
            }
        }
    }
}

/// Get the topic name for a subscriber
fn subscriber_topic(subscriber: &RustSubscriber) -> String {
    subscriber.topic.clone()
}

/// Check if a topic exists
fn topic_exists(topic: &str) -> bool {
    get_middleware().middleware.has_topic(topic)
}

/// List all topic names
fn list_topics() -> Vec<String> {
    get_middleware().middleware.topic_names()
}

/// Get subscriber count for a topic (returns 0 if topic doesn't exist)
fn subscriber_count(topic: &str) -> usize {
    get_middleware()
        .middleware
        .subscriber_count::<Vec<u8>>(topic)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_middleware_init() {
        assert!(middleware_init());
        assert!(middleware_init()); // Idempotent
    }

    #[test]
    fn test_publish_subscribe_polling() {
        middleware_init();

        let mut sub = create_subscriber("poll_test", QosKind::KeepLast, 16);
        let pub_ = create_publisher("poll_test", QosKind::KeepLast, 16);

        // Nothing to receive yet
        let result = subscriber_try_recv(&mut sub);
        assert!(!result.has_message);
        assert!(!result.closed);

        // Publish a message
        let publish_result = publish_bytes(&pub_, b"hello from rust");
        assert!(publish_result.success);
        assert!(publish_result.receivers >= 1);

        // Now we should receive it
        let result = subscriber_try_recv(&mut sub);
        assert!(result.has_message);
        assert_eq!(result.data, b"hello from rust");
    }

    #[test]
    fn test_sensor_data_qos_polling() {
        middleware_init();

        let mut sub = create_subscriber("sensor_poll", QosKind::SensorData, 1);
        let pub_ = create_publisher("sensor_poll", QosKind::SensorData, 1);

        // Publish multiple messages rapidly
        for i in 0..10u8 {
            publish_bytes(&pub_, &[i]);
        }

        // SensorData should give us the latest, skipping lagged
        let result = subscriber_try_recv(&mut sub);
        assert!(result.has_message);
        // Should get one of the later messages (exact value depends on timing)
        assert!(result.data[0] >= 5); // At least message 5 or later
    }

    #[test]
    fn test_topic_utilities() {
        middleware_init();

        let _pub = create_publisher("util_test2", QosKind::KeepLast, 16);
        
        // Need to create subscriber to register topic
        let _sub = create_subscriber("util_test2", QosKind::KeepLast, 16);

        assert!(topic_exists("util_test2"));
        assert!(list_topics().contains(&"util_test2".to_string()));
        assert!(subscriber_count("util_test2") >= 1);
    }

    #[test]
    fn test_subscriber_topic_name() {
        middleware_init();
        let sub = create_subscriber("name_test", QosKind::KeepLast, 16);
        assert_eq!(subscriber_topic(&sub), "name_test");
    }
}
