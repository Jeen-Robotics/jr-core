//! CXX bridge definitions for C++ interop
//!
//! Provides two consumption patterns:
//! 1. **Polling**: `subscriber_try_recv()` for non-blocking checks
//! 2. **Event-driven**: `subscriber_get_fd()` returns an eventfd that becomes
//!    readable when messages arrive. Use with epoll/select, then call `subscriber_recv()`.
//!
//! Messages are passed as serialized bytes for cross-language compatibility.
//!
//! # Lifecycle
//!
//! The global middleware is initialized on first use and lives until process exit.
//! Call `middleware_init()` once at startup.

use std::sync::Arc;
use std::os::fd::RawFd;

use once_cell::sync::OnceCell;
use tokio::runtime::Runtime;
use tokio::sync::{broadcast, mpsc};

use crate::{Middleware, Qos};

/// Global middleware instance
static MIDDLEWARE: OnceCell<Arc<MiddlewareHandle>> = OnceCell::new();

/// Handle that owns the middleware and tokio runtime
struct MiddlewareHandle {
    middleware: Middleware,
    /// Keeps the tokio runtime alive for the lifetime of the middleware.
    _runtime: Runtime,
}

impl MiddlewareHandle {
    fn runtime(&self) -> &Runtime {
        &self._runtime
    }
}

/// Get or initialize the global middleware
fn get_middleware() -> &'static Arc<MiddlewareHandle> {
    MIDDLEWARE.get_or_init(|| {
        // Multi-thread runtime is needed for subscriber eventfd forwarding tasks
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
/// Note: FFI delivery involves a copy on publish (.to_vec()) and receive (.clone()).
/// Zero-copy only applies to Rust-native typed subscriptions.
pub struct RustPublisher {
    /// Cached sender for zero-lookup publishing
    sender: Option<broadcast::Sender<Arc<Vec<u8>>>>,
    /// Error message if creation failed (e.g., type mismatch)
    error: Option<String>,
}

/// Subscriber handle for C++ with eventfd notification
pub struct RustSubscriber {
    topic: String,
    #[allow(dead_code)]  // Stored for potential future use (debugging, QoS changes)
    qos: Qos,
    /// Channel to receive messages from background task
    msg_rx: mpsc::Receiver<Arc<Vec<u8>>>,
    /// Eventfd for epoll notification (-1 if not available)
    event_fd: RawFd,
    /// Error message if creation failed
    error: Option<String>,
    /// Handle to stop the background task
    _stop_tx: Option<mpsc::Sender<()>>,
}

impl Drop for RustSubscriber {
    fn drop(&mut self) {
        // Close eventfd if open
        #[cfg(target_os = "linux")]
        if self.event_fd >= 0 {
            unsafe { libc::close(self.event_fd) };
        }
        // _stop_tx drop will signal the background task to stop
    }
}

/// Buffer size for message channel (between tokio task and C++)
const MESSAGE_BUFFER_SIZE: usize = 64;

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
        fn middleware_init() -> bool;

        // Publisher API
        /// Create a publisher. For SensorData QoS, capacity is ignored (always 1).
        fn create_publisher(topic: &str, qos: QosKind, capacity: usize) -> Box<RustPublisher>;
        fn publisher_is_valid(publisher: &RustPublisher) -> bool;
        fn publish_bytes(publisher: &RustPublisher, data: &[u8]) -> PublishResult;

        // Subscriber API
        /// Create a subscriber. For SensorData QoS, capacity is ignored (always 1).
        fn create_subscriber(topic: &str, qos: QosKind, capacity: usize) -> Box<RustSubscriber>;
        fn subscriber_is_valid(subscriber: &RustSubscriber) -> bool;
        
        /// Get eventfd for epoll/select integration.
        /// Returns -1 if not available (non-Linux or error).
        /// The fd becomes readable when messages are available.
        fn subscriber_get_fd(subscriber: &RustSubscriber) -> i32;
        
        /// Non-blocking receive. Use after epoll signals the fd is readable,
        /// or for polling without epoll.
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
/// Note: For SensorData, capacity is ignored (always uses 1)
fn qos_from_ffi(kind: QosKind, capacity: usize) -> Qos {
    match kind {
        QosKind::KeepLast => Qos::KeepLast(capacity),
        QosKind::SensorData => Qos::SensorData,  // capacity ignored, always 1
        // Defense-in-depth fallback (CXX validates enums at bridge level)
        _ => Qos::KeepLast(capacity),
    }
}

/// Create an eventfd (Linux only)
#[cfg(target_os = "linux")]
fn create_eventfd() -> RawFd {
    // Use raw libc for simplicity - we manage the fd lifetime ourselves
    unsafe {
        libc::eventfd(0, libc::EFD_NONBLOCK | libc::EFD_CLOEXEC)
    }
}

#[cfg(not(target_os = "linux"))]
fn create_eventfd() -> RawFd {
    -1  // Not available on non-Linux
}

/// Signal eventfd that a message is available
#[cfg(target_os = "linux")]
fn signal_eventfd(fd: RawFd) {
    if fd >= 0 {
        let val: u64 = 1;
        unsafe {
            libc::write(fd, &val as *const u64 as *const libc::c_void, 8);
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn signal_eventfd(_fd: RawFd) {
    // No-op on non-Linux
}

/// Clear eventfd (call after reading messages)
#[cfg(target_os = "linux")]
fn clear_eventfd(fd: RawFd) {
    if fd >= 0 {
        let mut val: u64 = 0;
        unsafe {
            libc::read(fd, &mut val as *mut u64 as *mut libc::c_void, 8);
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn clear_eventfd(_fd: RawFd) {
    // No-op on non-Linux
}

/// Initialize the middleware (idempotent)
fn middleware_init() -> bool {
    get_middleware();
    true
}

/// Create a publisher for a topic
fn create_publisher(topic: &str, qos: QosKind, capacity: usize) -> Box<RustPublisher> {
    let qos = qos_from_ffi(qos, capacity);
    let handle = get_middleware();

    match handle.middleware.get_or_create_sender_raw(topic, qos) {
        Some(sender) => Box::new(RustPublisher {
            sender: Some(sender),
            error: None,
        }),
        None => Box::new(RustPublisher {
            sender: None,
            error: Some(format!(
                "Failed to create publisher for '{}': topic exists with different type",
                topic
            )),
        }),
    }
}

/// Check if a publisher is valid
fn publisher_is_valid(publisher: &RustPublisher) -> bool {
    publisher.sender.is_some()
}

/// Publish serialized bytes to a topic
fn publish_bytes(publisher: &RustPublisher, data: &[u8]) -> PublishResult {
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

    let msg = Arc::new(data.to_vec());

    match sender.send(msg) {
        Ok(receivers) => PublishResult {
            success: true,
            receivers,
            error_msg: String::new(),
        },
        // No receivers is normal in pub/sub (startup, no consumers yet)
        // Return success:true with receivers:0 to match Middleware::publish semantics
        Err(_) => PublishResult {
            success: true,
            receivers: 0,
            error_msg: String::new(),
        },
    }
}

/// Create a subscriber with eventfd notification
fn create_subscriber(topic: &str, qos: QosKind, capacity: usize) -> Box<RustSubscriber> {
    let handle = get_middleware();
    let qos_internal = qos_from_ffi(qos, capacity);

    // Try to subscribe
    let broadcast_rx = match handle.middleware.subscribe_raw(topic, qos_internal) {
        Some(rx) => rx,
        None => {
            return Box::new(RustSubscriber {
                topic: topic.to_string(),
                qos: qos_internal,
                msg_rx: mpsc::channel(1).1,  // dummy channel
                event_fd: -1,
                error: Some(format!(
                    "Failed to subscribe to '{}': topic exists with different type",
                    topic
                )),
                _stop_tx: None,
            });
        }
    };

    // Create eventfd for notification
    let event_fd = create_eventfd();

    // Create channel for messages
    let (msg_tx, msg_rx) = mpsc::channel(MESSAGE_BUFFER_SIZE);
    let (stop_tx, mut stop_rx) = mpsc::channel::<()>(1);

    // Spawn background task to forward messages and signal eventfd
    let event_fd_copy = event_fd;
    let skip_lagged = qos_internal.skip_lagged();
    
    handle.runtime().spawn(async move {
        let mut rx = broadcast_rx;
        
        loop {
            tokio::select! {
                biased;
                
                _ = stop_rx.recv() => {
                    break;
                }
                
                result = rx.recv() => {
                    match result {
                        Ok(msg) => {
                            // Send to C++ channel
                            if msg_tx.send(msg).await.is_err() {
                                break;  // Receiver dropped
                            }
                            // Signal eventfd
                            signal_eventfd(event_fd_copy);
                        }
                        Err(broadcast::error::RecvError::Lagged(_)) => {
                            if skip_lagged {
                                continue;  // SensorData: skip and retry
                            }
                            // For KeepLast, we lost messages but continue
                        }
                        Err(broadcast::error::RecvError::Closed) => {
                            break;
                        }
                    }
                }
            }
        }
    });

    Box::new(RustSubscriber {
        topic: topic.to_string(),
        qos: qos_internal,
        msg_rx,
        event_fd,
        error: None,
        _stop_tx: Some(stop_tx),
    })
}

/// Check if a subscriber is valid
fn subscriber_is_valid(subscriber: &RustSubscriber) -> bool {
    subscriber.error.is_none()
}

/// Get the eventfd for epoll/select integration
fn subscriber_get_fd(subscriber: &RustSubscriber) -> i32 {
    subscriber.event_fd
}

/// Non-blocking receive
fn subscriber_try_recv(subscriber: &mut RustSubscriber) -> RecvResult {
    if subscriber.error.is_some() {
        return RecvResult {
            has_message: false,
            data: Vec::new(),
            closed: true,
            lagged: 0,
            error_msg: subscriber.error.clone().unwrap_or_default(),
        };
    }

    // Clear eventfd first (read the counter)
    clear_eventfd(subscriber.event_fd);

    // Try to receive from channel
    match subscriber.msg_rx.try_recv() {
        Ok(msg) => RecvResult {
            has_message: true,
            data: (*msg).clone(),
            closed: false,
            lagged: 0,
            error_msg: String::new(),
        },
        Err(mpsc::error::TryRecvError::Empty) => RecvResult {
            has_message: false,
            data: Vec::new(),
            closed: false,
            lagged: 0,
            error_msg: String::new(),
        },
        Err(mpsc::error::TryRecvError::Disconnected) => RecvResult {
            has_message: false,
            data: Vec::new(),
            closed: true,
            lagged: 0,
            error_msg: "Channel closed".to_string(),
        },
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

/// Get subscriber count for a topic
fn subscriber_count(topic: &str) -> usize {
    get_middleware()
        .middleware
        .subscriber_count::<Vec<u8>>(topic)
        .unwrap_or(0)
}

#[cfg(feature = "test-hooks")]
pub mod test_hooks {
    use super::*;

    pub type TestPublisher = RustPublisher;
    pub type TestSubscriber = RustSubscriber;

    pub fn middleware_init_for_tests() -> bool {
        middleware_init()
    }

    pub fn create_publisher_for_tests(
        topic: &str,
        qos: QosKind,
        capacity: usize,
    ) -> Box<TestPublisher> {
        create_publisher(topic, qos, capacity)
    }

    pub fn publisher_is_valid_for_tests(publisher: &TestPublisher) -> bool {
        publisher_is_valid(publisher)
    }

    pub fn publish_bytes_for_tests(publisher: &TestPublisher, data: &[u8]) -> PublishResult {
        publish_bytes(publisher, data)
    }

    pub fn create_subscriber_for_tests(
        topic: &str,
        qos: QosKind,
        capacity: usize,
    ) -> Box<TestSubscriber> {
        create_subscriber(topic, qos, capacity)
    }

    pub fn subscriber_is_valid_for_tests(subscriber: &TestSubscriber) -> bool {
        subscriber_is_valid(subscriber)
    }

    pub fn subscriber_get_fd_for_tests(subscriber: &TestSubscriber) -> i32 {
        subscriber_get_fd(subscriber)
    }

    pub fn subscriber_try_recv_for_tests(subscriber: &mut TestSubscriber) -> RecvResult {
        subscriber_try_recv(subscriber)
    }

    pub fn subscriber_topic_for_tests(subscriber: &TestSubscriber) -> String {
        subscriber_topic(subscriber)
    }

    pub fn topic_exists_for_tests(topic: &str) -> bool {
        topic_exists(topic)
    }

    pub fn list_topics_for_tests() -> Vec<String> {
        list_topics()
    }

    pub fn subscriber_count_for_tests(topic: &str) -> usize {
        subscriber_count(topic)
    }

    pub fn force_string_topic_for_tests(topic: &str) {
        let _ = get_middleware()
            .middleware
            .subscribe_with_qos::<String>(topic, Qos::default());
    }
}
