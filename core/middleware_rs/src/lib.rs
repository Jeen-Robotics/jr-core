//! Rust Middleware - Zero-copy pub/sub for inter-thread communication
//!
//! This crate provides a high-performance pub/sub middleware with:
//! - Zero-copy message passing via Arc<T>
//! - Async API built on tokio
//! - Type-safe topics with runtime type checking
//! - C++ bindings via CXX (Phase 2)
//!
//! # Quick Start
//!
//! ```ignore
//! use middleware_rs::Middleware;
//! use std::sync::Arc;
//!
//! #[tokio::main]
//! async fn main() {
//!     let mw = Middleware::new();
//!     
//!     // Subscribe to a topic
//!     let mut sub = mw.subscribe::<String>("greetings").await.unwrap();
//!     
//!     // Publish a message
//!     mw.publish("greetings", Arc::new("Hello!".to_string())).await.unwrap();
//!     
//!     // Receive the message (zero-copy - same Arc)
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

/// Generated protobuf message types (compatible with C++ protobuf wire format)
/// 
/// Proto definitions come from jr-msgs submodule (shared with C++).
/// Only bag.proto is middleware-specific and kept locally.
pub mod proto {
    /// Standard messages (Header, Time, Duration)
    pub mod std_msgs {
        include!(concat!(env!("OUT_DIR"), "/std_msgs.rs"));
    }
    
    /// Geometry messages (Vector3, Point, Quaternion, Transform, etc.)
    pub mod geometry_msgs {
        include!(concat!(env!("OUT_DIR"), "/geometry_msgs.rs"));
    }
    
    /// Sensor messages (Image, Imu, CameraInfo)
    /// Uses types from std_msgs and geometry_msgs
    pub mod sensor_msgs {
        // Re-export dependencies for generated code
        pub use super::std_msgs;
        pub use super::geometry_msgs;
        
        include!(concat!(env!("OUT_DIR"), "/sensor_msgs.rs"));
    }
    
    /// Bag messages (BagHeader, BagRecord, VideoFrameReference)
    /// Middleware-specific, not from jr-msgs
    pub mod jr {
        pub mod mw {
            include!(concat!(env!("OUT_DIR"), "/jr.mw.rs"));
        }
    }
}

pub use middleware::{Middleware, MiddlewareError, Subscription};
pub use topic::TopicRegistry;

#[cfg(test)]
mod integration_tests {
    use super::*;
    use std::sync::Arc;
    use tokio::time::{timeout, Duration};

    /// Test that demonstrates the full pub/sub workflow
    #[tokio::test]
    async fn test_full_pubsub_workflow() {
        let mw = Middleware::new();

        // Create multiple subscribers before publishing
        let mut sub1 = mw.subscribe::<String>("workflow").await.unwrap();
        let mut sub2 = mw.subscribe::<String>("workflow").await.unwrap();

        // Publish a message
        let message = Arc::new("workflow test message".to_string());
        let receiver_count = mw.publish("workflow", message.clone()).await.unwrap();
        assert_eq!(receiver_count, 2);

        // Both subscribers should receive the message
        let r1 = sub1.recv().await.unwrap();
        let r2 = sub2.recv().await.unwrap();

        assert_eq!(*r1, "workflow test message");
        assert_eq!(*r2, "workflow test message");

        // Verify zero-copy - both Arc point to the same allocation
        assert!(Arc::ptr_eq(&r1, &r2));
    }

    /// Test with complex message types
    #[derive(Debug, Clone, PartialEq)]
    struct CameraFrame {
        width: u32,
        height: u32,
        data: Vec<u8>,
        timestamp_ns: u64,
    }

    #[tokio::test]
    async fn test_complex_message_types() {
        let mw = Middleware::new();

        let mut sub = mw.subscribe::<CameraFrame>("camera/front").await.unwrap();

        // Simulate a large camera frame
        let frame = CameraFrame {
            width: 1920,
            height: 1080,
            data: vec![0u8; 1920 * 1080 * 3], // RGB data
            timestamp_ns: 1234567890,
        };

        let frame_arc = Arc::new(frame);
        let original_ptr = Arc::as_ptr(&frame_arc);

        mw.publish("camera/front", frame_arc).await.unwrap();

        let received = sub.recv().await.unwrap();

        // Verify zero-copy for large data
        assert_eq!(Arc::as_ptr(&received), original_ptr);
        assert_eq!(received.width, 1920);
        assert_eq!(received.height, 1080);
        assert_eq!(received.data.len(), 1920 * 1080 * 3);
    }

    /// Test concurrent publish/subscribe from multiple tasks
    #[tokio::test]
    async fn test_concurrent_pubsub() {
        let mw = Arc::new(Middleware::new());
        let message_count = 100;

        // Spawn subscriber tasks
        let mut handles = vec![];

        for sub_id in 0..3 {
            let mw_clone = mw.clone();
            let handle = tokio::spawn(async move {
                let mut sub = mw_clone.subscribe::<i32>("concurrent").await.unwrap();
                let mut received = vec![];

                for _ in 0..message_count {
                    match timeout(Duration::from_secs(5), sub.recv()).await {
                        Ok(Ok(msg)) => received.push(*msg),
                        Ok(Err(e)) => panic!("Subscriber {} error: {:?}", sub_id, e),
                        Err(_) => panic!("Subscriber {} timeout", sub_id),
                    }
                }

                received
            });
            handles.push(handle);
        }

        // Give subscribers time to set up
        tokio::time::sleep(Duration::from_millis(10)).await;

        // Spawn publisher task
        let mw_pub = mw.clone();
        let pub_handle = tokio::spawn(async move {
            for i in 0..message_count {
                mw_pub.publish_owned("concurrent", i as i32).await.unwrap();
            }
        });

        // Wait for publisher
        pub_handle.await.unwrap();

        // Wait for all subscribers and verify results
        for handle in handles {
            let received = handle.await.unwrap();
            assert_eq!(received.len(), message_count);
            // Verify order
            for (i, &msg) in received.iter().enumerate() {
                assert_eq!(msg, i as i32);
            }
        }
    }

    /// Test that dropping subscription (unsubscribe) works correctly
    #[tokio::test]
    async fn test_unsubscribe_behavior() {
        let mw = Middleware::new();

        // Create two subscribers
        let mut sub1 = mw.subscribe::<String>("unsub").await.unwrap();
        let sub2 = mw.subscribe::<String>("unsub").await.unwrap();

        assert_eq!(mw.subscriber_count::<String>("unsub"), Some(2));

        // Drop sub2 (unsubscribe)
        drop(sub2);

        // Now only one subscriber
        assert_eq!(mw.subscriber_count::<String>("unsub"), Some(1));

        // Publish should deliver to remaining subscriber
        mw.publish_owned("unsub", "still here".to_string())
            .await
            .unwrap();

        let msg = sub1.recv().await.unwrap();
        assert_eq!(*msg, "still here");

        // Drop last subscriber
        drop(sub1);
        assert_eq!(mw.subscriber_count::<String>("unsub"), Some(0));

        // Publishing to empty topic should still work
        let count = mw
            .publish_owned("unsub", "nobody listening".to_string())
            .await
            .unwrap();
        assert_eq!(count, 0);
    }

    /// Test multiple independent topics
    #[tokio::test]
    async fn test_multiple_independent_topics() {
        let mw = Middleware::new();

        // Use unique topic names to avoid conflicts with other tests
        let mut cam_sub = mw.subscribe::<Vec<u8>>("multi_topics/camera").await.unwrap();
        let mut imu_sub = mw.subscribe::<(f64, f64, f64)>("multi_topics/imu").await.unwrap();
        let mut gps_sub = mw.subscribe::<(f64, f64)>("multi_topics/gps").await.unwrap();

        // Publish to different topics
        mw.publish_owned("multi_topics/camera", vec![1u8, 2u8, 3u8]).await.unwrap();
        mw.publish_owned("multi_topics/imu", (1.0, 2.0, 3.0)).await.unwrap();
        mw.publish_owned("multi_topics/gps", (55.7558, 37.6173)).await.unwrap();

        // Each subscriber only gets its topic's messages
        let cam = cam_sub.recv().await.unwrap();
        let imu = imu_sub.recv().await.unwrap();
        let gps = gps_sub.recv().await.unwrap();

        assert_eq!(*cam, vec![1, 2, 3]);
        assert_eq!(*imu, (1.0, 2.0, 3.0));
        assert_eq!(*gps, (55.7558, 37.6173));
    }

    /// Verify zero-copy by checking Arc strong_count
    #[tokio::test]
    async fn test_zero_copy_arc_counting() {
        let mw = Middleware::new();

        let mut sub1 = mw.subscribe::<String>("arc_test").await.unwrap();
        let mut sub2 = mw.subscribe::<String>("arc_test").await.unwrap();

        let msg = Arc::new("shared data".to_string());
        assert_eq!(Arc::strong_count(&msg), 1); // Only original

        mw.publish("arc_test", msg.clone()).await.unwrap();

        // Now msg has been cloned into the broadcast channel
        // strong_count includes: original + channel internal copies

        let r1 = sub1.recv().await.unwrap();
        let r2 = sub2.recv().await.unwrap();

        // r1 and r2 are the same Arc as msg (zero-copy)
        assert!(Arc::ptr_eq(&r1, &msg));
        assert!(Arc::ptr_eq(&r2, &msg));
        assert!(Arc::ptr_eq(&r1, &r2));

        // Total count should be: original (msg) + r1 + r2 = 3
        // (channel may have released its copy after send)
        assert!(Arc::strong_count(&msg) >= 3);
    }
}
