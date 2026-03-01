//! CXX bridge definitions for C++ interop
//!
//! Uses a polling API for subscribers instead of callbacks (CXX limitation).
//! C++ code calls `subscriber_try_recv()` to poll for messages.
//!
//! Messages are passed as serialized bytes for cross-language compatibility.
//!
//! # Lifecycle
//!
//! The global middleware is initialized on first use and lives until process exit.
//! Call `middleware_init()` once at startup. QoS/capacity cannot be reconfigured
//! after initialization.

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
    /// Keeps the tokio runtime alive for the lifetime of the middleware.
    /// Must not be dropped while the middleware is active.
    _runtime: Runtime,
}

/// Get or initialize the global middleware
fn get_middleware() -> &'static Arc<MiddlewareHandle> {
    MIDDLEWARE.get_or_init(|| {
        let runtime = Runtime::new().expect("Failed to create tokio runtime");
        Arc::new(MiddlewareHandle {
            middleware: Middleware::new(),
            _runtime: runtime,
        })
    })
}

/// Publisher handle for C++
///
/// Caches the broadcast sender for efficient high-frequency publishing.
pub struct RustPublisher {
    #[allow(dead_code)] // Kept for debugging/logging
    topic: String,
    /// Cached sender for zero-lookup publishing
    sender: Option<broadcast::Sender<Arc<Vec<u8>>>>,
    /// Error message if creation failed (e.g., type mismatch)
    error: Option<String>,
}

/// Subscriber handle for C++ (polling-based)
pub struct RustSubscriber {
    topic: String,
    receiver: Option<broadcast::Receiver<Arc<Vec<u8>>>>,
    qos: Qos,
    /// Error message if creation failed
    error: Option<String>,
}

/// Maximum retry attempts for SensorData QoS when lagged
const SENSOR_DATA_MAX_RETRIES: usize = 5;

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
        /// True if the channel is closed or subscriber is invalid
        pub closed: bool,
        /// Number of messages that were skipped (lagged)
        pub lagged: u64,
        /// Error message (non-empty if an error occurred)
        pub error_msg: String,
    }

    extern "Rust" {
        // Opaque Rust types
        type RustPublisher;
        type RustSubscriber;

        // Middleware lifecycle
        /// Initialize the global middleware singleton.
        /// Idempotent - safe to call multiple times.
        /// The middleware lives until process exit.
        fn middleware_init() -> bool;

        // Publisher API
        fn create_publisher(topic: &str, qos: QosKind, capacity: usize) -> Box<RustPublisher>;
        fn publisher_is_valid(publisher: &RustPublisher) -> bool;
        fn publish_bytes(publisher: &RustPublisher, data: &[u8]) -> PublishResult;

        // Subscriber API (polling-based)
        fn create_subscriber(topic: &str, qos: QosKind, capacity: usize) -> Box<RustSubscriber>;
        fn subscriber_is_valid(subscriber: &RustSubscriber) -> bool;
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
        // CXX enums are repr(u8), so we need a fallback for invalid values
        // that could come from C++ side. Default to KeepLast.
        _ => Qos::KeepLast(capacity),
    }
}

/// Initialize the middleware (idempotent)
fn middleware_init() -> bool {
    get_middleware();
    true
}

/// Create a publisher for a topic
///
/// The sender is cached for efficient high-frequency publishing.
/// Check `publisher_is_valid()` before use - creation may fail if
/// the topic already exists with a different type.
fn create_publisher(topic: &str, qos: QosKind, capacity: usize) -> Box<RustPublisher> {
    let qos = qos_from_ffi(qos, capacity);
    let handle = get_middleware();

    // Get or create sender - this also registers the topic type
    match handle
        .middleware
        .get_or_create_sender_raw(topic, qos)
    {
        Some(sender) => Box::new(RustPublisher {
            topic: topic.to_string(),
            sender: Some(sender),
            error: None,
        }),
        None => Box::new(RustPublisher {
            topic: topic.to_string(),
            sender: None,
            error: Some(format!(
                "Failed to create publisher for '{}': topic exists with different type",
                topic
            )),
        }),
    }
}

/// Check if a publisher is valid (was created successfully)
fn publisher_is_valid(publisher: &RustPublisher) -> bool {
    publisher.sender.is_some()
}

/// Publish serialized bytes to a topic
fn publish_bytes(publisher: &RustPublisher, data: &[u8]) -> PublishResult {
    // Check if publisher is valid
    let sender = match &publisher.sender {
        Some(s) => s,
        None => {
            return PublishResult {
                success: false,
                receivers: 0,
                error_msg: publisher
                    .error
                    .clone()
                    .unwrap_or_else(|| "Invalid publisher".to_string()),
            };
        }
    };

    // Create a Vec from the data (copy required for FFI boundary)
    let msg = Arc::new(data.to_vec());

    match sender.send(msg) {
        Ok(receivers) => PublishResult {
            success: true,
            receivers,
            error_msg: String::new(),
        },
        Err(_) => PublishResult {
            success: true,  // Send "succeeded" but no receivers
            receivers: 0,
            error_msg: String::new(),
        },
    }
}

/// Create a subscriber for a topic (polling-based)
///
/// Returns a subscriber handle. Check `subscriber_is_valid()` before use.
/// If creation failed (e.g., type mismatch), the subscriber will be invalid
/// and `subscriber_try_recv()` will return an error.
fn create_subscriber(topic: &str, qos: QosKind, capacity: usize) -> Box<RustSubscriber> {
    let handle = get_middleware();
    let qos = qos_from_ffi(qos, capacity);

    // Try to subscribe - this may fail if topic exists with different type
    match handle
        .middleware
        .subscribe_raw(topic, qos)
    {
        Some(receiver) => Box::new(RustSubscriber {
            topic: topic.to_string(),
            receiver: Some(receiver),
            qos,
            error: None,
        }),
        None => Box::new(RustSubscriber {
            topic: topic.to_string(),
            receiver: None,
            qos,
            error: Some(format!(
                "Failed to subscribe to '{}': topic exists with different type",
                topic
            )),
        }),
    }
}

/// Check if a subscriber is valid (was created successfully)
fn subscriber_is_valid(subscriber: &RustSubscriber) -> bool {
    subscriber.receiver.is_some()
}

/// Try to receive a message without blocking
///
/// For SensorData QoS, lagged messages are silently skipped (up to 5 retries).
fn subscriber_try_recv(subscriber: &mut RustSubscriber) -> RecvResult {
    // Check if subscriber is valid
    let receiver = match &mut subscriber.receiver {
        Some(r) => r,
        None => {
            return RecvResult {
                has_message: false,
                data: Vec::new(),
                closed: true,
                lagged: 0,
                error_msg: subscriber
                    .error
                    .clone()
                    .unwrap_or_else(|| "Invalid subscriber".to_string()),
            };
        }
    };

    // Retry loop for SensorData (capped to prevent infinite spinning)
    let max_retries = if subscriber.qos.skip_lagged() {
        SENSOR_DATA_MAX_RETRIES
    } else {
        1
    };

    let mut last_lagged = 0u64;

    for _ in 0..max_retries {
        match receiver.try_recv() {
            Ok(msg) => {
                return RecvResult {
                    has_message: true,
                    data: (*msg).clone(),
                    closed: false,
                    lagged: 0,
                    error_msg: String::new(),
                };
            }
            Err(broadcast::error::TryRecvError::Empty) => {
                return RecvResult {
                    has_message: false,
                    data: Vec::new(),
                    closed: false,
                    lagged: 0,
                    error_msg: String::new(),
                };
            }
            Err(broadcast::error::TryRecvError::Lagged(n)) => {
                last_lagged = n;
                if subscriber.qos.skip_lagged() {
                    // SensorData: retry to get latest
                    continue;
                }
                return RecvResult {
                    has_message: false,
                    data: Vec::new(),
                    closed: false,
                    lagged: n,
                    error_msg: String::new(),
                };
            }
            Err(broadcast::error::TryRecvError::Closed) => {
                return RecvResult {
                    has_message: false,
                    data: Vec::new(),
                    closed: true,
                    lagged: 0,
                    error_msg: String::new(),
                };
            }
        }
    }

    // Max retries exhausted for SensorData - return lagged result
    RecvResult {
        has_message: false,
        data: Vec::new(),
        closed: false,
        lagged: last_lagged,
        error_msg: String::new(),
    }
}

/// Get the topic name for a subscriber
///
/// Note: C++ callers should cache this result rather than calling in a loop.
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
///
/// Note: This counts only FFI (Vec<u8>) subscribers. Rust-native typed
/// subscribers on the same topic name but different type are not counted.
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

        let mut sub = create_subscriber("ffi_poll_test2", QosKind::KeepLast, 16);
        assert!(subscriber_is_valid(&sub));

        let pub_ = create_publisher("ffi_poll_test2", QosKind::KeepLast, 16);
        assert!(publisher_is_valid(&pub_));

        // Nothing to receive yet
        let result = subscriber_try_recv(&mut sub);
        assert!(!result.has_message);
        assert!(!result.closed);
        assert!(result.error_msg.is_empty());

        // Publish a message
        let publish_result = publish_bytes(&pub_, b"hello from rust ffi v2");
        assert!(publish_result.success);
        assert!(publish_result.receivers >= 1);

        // Now we should receive it
        let result = subscriber_try_recv(&mut sub);
        assert!(result.has_message);
        assert_eq!(result.data, b"hello from rust ffi v2");
    }

    #[test]
    fn test_sensor_data_qos_polling() {
        middleware_init();

        let mut sub = create_subscriber("ffi_sensor_poll2", QosKind::SensorData, 1);
        assert!(subscriber_is_valid(&sub));

        let pub_ = create_publisher("ffi_sensor_poll2", QosKind::SensorData, 1);
        assert!(publisher_is_valid(&pub_));

        // Publish multiple messages rapidly (capacity=1, so older ones are dropped)
        for i in 0..10u8 {
            publish_bytes(&pub_, &[i]);
        }

        // SensorData should give us the latest message (9)
        let result = subscriber_try_recv(&mut sub);
        assert!(result.has_message);
        assert_eq!(result.data[0], 9); // Must be exactly the last message
    }

    #[test]
    fn test_topic_utilities() {
        middleware_init();

        let pub_ = create_publisher("ffi_util_test2", QosKind::KeepLast, 16);
        assert!(publisher_is_valid(&pub_));

        let _sub = create_subscriber("ffi_util_test2", QosKind::KeepLast, 16);

        assert!(topic_exists("ffi_util_test2"));
        assert!(list_topics().contains(&"ffi_util_test2".to_string()));
        assert!(subscriber_count("ffi_util_test2") >= 1);
    }

    #[test]
    fn test_subscriber_topic_name() {
        middleware_init();
        let sub = create_subscriber("ffi_name_test2", QosKind::KeepLast, 16);
        assert_eq!(subscriber_topic(&sub), "ffi_name_test2");
    }

    #[test]
    fn test_invalid_subscriber_returns_error() {
        middleware_init();

        // Create a topic with String type via FFI-internal method
        // (simulates conflict with Rust-native code)
        let mw = &get_middleware().middleware;
        let _ = mw.subscribe_with_qos::<String>("ffi_type_conflict2", Qos::default());

        // Try to create FFI subscriber with Vec<u8> type - should fail
        let mut sub = create_subscriber("ffi_type_conflict2", QosKind::KeepLast, 16);
        assert!(!subscriber_is_valid(&sub));

        // try_recv should return error
        let result = subscriber_try_recv(&mut sub);
        assert!(!result.has_message);
        assert!(result.closed);
        assert!(!result.error_msg.is_empty());
    }

    #[test]
    fn test_invalid_publisher_returns_error() {
        middleware_init();

        // Create a topic with String type
        let mw = &get_middleware().middleware;
        let _ = mw.subscribe_with_qos::<String>("ffi_pub_conflict", Qos::default());

        // Try to create FFI publisher with Vec<u8> type - should fail
        let pub_ = create_publisher("ffi_pub_conflict", QosKind::KeepLast, 16);
        assert!(!publisher_is_valid(&pub_));

        // publish should return error
        let result = publish_bytes(&pub_, b"test");
        assert!(!result.success);
        assert!(!result.error_msg.is_empty());
    }
}
