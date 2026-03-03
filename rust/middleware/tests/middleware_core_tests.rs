mod common;

use std::sync::Arc;
use std::time::Duration;

use middleware::{Middleware, MiddlewareError, Qos};

#[tokio::test]
async fn basic_pubsub() {
    let mw = Middleware::new();
    let topic = common::unique_topic("basic_pubsub");

    let mut sub = mw.subscribe::<String>(&topic).unwrap();
    let msg = Arc::new("Hello, World!".to_string());
    let count = mw.publish(&topic, msg.clone()).unwrap();
    assert_eq!(count, 1);

    let received = sub.recv().await.unwrap();
    assert_eq!(*received, "Hello, World!");
}

#[tokio::test]
async fn multiple_subscribers_same_arc() {
    let mw = Middleware::new();
    let topic = common::unique_topic("multi_sub_arc");

    let mut sub1 = mw.subscribe::<String>(&topic).unwrap();
    let mut sub2 = mw.subscribe::<String>(&topic).unwrap();
    let mut sub3 = mw.subscribe::<String>(&topic).unwrap();

    let msg = Arc::new("shared message".to_string());
    let original_ptr = Arc::as_ptr(&msg);
    let count = mw.publish(&topic, msg).unwrap();
    assert_eq!(count, 3);

    let r1 = sub1.recv().await.unwrap();
    let r2 = sub2.recv().await.unwrap();
    let r3 = sub3.recv().await.unwrap();

    assert_eq!(Arc::as_ptr(&r1), original_ptr);
    assert_eq!(Arc::as_ptr(&r2), original_ptr);
    assert_eq!(Arc::as_ptr(&r3), original_ptr);
}

#[tokio::test]
async fn unsubscribe_stops_delivery() {
    let mw = Middleware::new();
    let topic = common::unique_topic("unsub");

    let sub = mw.subscribe::<String>(&topic).unwrap();
    assert_eq!(mw.subscriber_count::<String>(&topic), Some(1));

    drop(sub);

    assert_eq!(mw.subscriber_count::<String>(&topic), Some(0));
    let result = mw.publish(&topic, Arc::new("nobody".to_string())).unwrap();
    assert_eq!(result, 0);
}

#[tokio::test]
async fn type_mismatch() {
    let mw = Middleware::new();
    let topic = common::unique_topic("typed");

    let _sub = mw.subscribe::<String>(&topic).unwrap();

    let publish_result = mw.publish(&topic, Arc::new(42i32));
    assert!(matches!(
        publish_result,
        Err(MiddlewareError::TypeMismatch { .. })
    ));

    let subscribe_result = mw.subscribe::<i32>(&topic);
    assert!(matches!(
        subscribe_result,
        Err(MiddlewareError::TypeMismatch { .. })
    ));
}

#[tokio::test]
async fn publish_owned_and_try_recv() {
    let mw = Middleware::new();
    let topic = common::unique_topic("owned_try_recv");

    let mut sub = mw.subscribe::<String>(&topic).unwrap();

    assert!(sub.try_recv().is_none());

    mw.publish_owned(&topic, "auto-wrapped".to_string()).unwrap();

    let result = sub.try_recv();
    assert!(result.is_some());
    assert_eq!(*result.unwrap().unwrap(), "auto-wrapped");
    assert!(sub.try_recv().is_none());
}

#[tokio::test]
async fn multiple_messages_ordered() {
    let mw = Middleware::new();
    let topic = common::unique_topic("multi_messages");

    let mut sub = mw.subscribe::<i32>(&topic).unwrap();

    for i in 0..5 {
        mw.publish_owned(&topic, i).unwrap();
    }

    for i in 0..5 {
        let msg = sub.recv().await.unwrap();
        assert_eq!(*msg, i);
    }
}

#[tokio::test]
async fn topic_names_and_remove_topic() {
    let mw = Middleware::new();
    let topic1 = common::unique_topic("topic1");
    let topic2 = common::unique_topic("topic2");

    mw.subscribe::<String>(&topic1).unwrap();
    mw.subscribe::<i32>(&topic2).unwrap();

    let names = mw.topic_names();
    assert!(names.contains(&topic1));
    assert!(names.contains(&topic2));

    assert!(mw.remove_topic(&topic1));
    assert!(!mw.has_topic(&topic1));
}

#[tokio::test]
async fn resubscribe_starts_fresh() {
    let mw = Middleware::new();
    let topic = common::unique_topic("resub");

    let sub1 = mw.subscribe::<String>(&topic).unwrap();

    mw.publish_owned(&topic, "first".to_string()).unwrap();

    let mut sub2 = sub1.resubscribe();

    mw.publish_owned(&topic, "second".to_string()).unwrap();

    let msg = sub2.recv().await.unwrap();
    assert_eq!(*msg, "second");
}

#[tokio::test]
async fn sensor_data_qos_returns_latest() {
    let mw = Middleware::new();
    let topic = common::unique_topic("sensor");

    let mut sub = mw
        .subscribe_with_qos::<i32>(&topic, Qos::SensorData)
        .unwrap();

    assert_eq!(sub.qos(), Qos::SensorData);
    assert_eq!(mw.get_qos(&topic), Some(Qos::SensorData));

    for i in 0..10 {
        mw.publish_owned(&topic, i).unwrap();
    }

    let msg = sub.recv().await.unwrap();
    assert_eq!(*msg, 9);
}

#[tokio::test]
async fn keep_last_qos_buffered_delivery() {
    let mw = Middleware::new();
    let topic = common::unique_topic("buffered");

    let mut sub = mw
        .subscribe_with_qos::<i32>(&topic, Qos::KeepLast(4))
        .unwrap();

    for i in 0..3 {
        mw.publish_owned(&topic, i).unwrap();
    }

    for i in 0..3 {
        let msg = sub.recv().await.unwrap();
        assert_eq!(*msg, i);
    }
}

#[tokio::test]
async fn recv_returns_lagged_for_keep_last() {
    let mw = Middleware::new();
    let topic = common::unique_topic("lagged_recv");

    let mut sub = mw
        .subscribe_with_qos::<i32>(&topic, Qos::KeepLast(2))
        .unwrap();

    for i in 0..10 {
        mw.publish_owned(&topic, i).unwrap();
    }

    let err = sub.recv().await.unwrap_err();
    assert!(matches!(
        err,
        MiddlewareError::Lagged {
            topic: t,
            count: _
        } if t == topic
    ));
}

#[test]
fn try_recv_returns_lagged_for_keep_last() {
    let mw = Middleware::new();
    let topic = common::unique_topic("lagged_try_recv");

    let mut sub = mw
        .subscribe_with_qos::<i32>(&topic, Qos::KeepLast(2))
        .unwrap();

    for i in 0..10 {
        mw.publish_owned(&topic, i).unwrap();
    }

    let result = sub.try_recv();
    assert!(matches!(
        result,
        Some(Err(MiddlewareError::Lagged {
            topic: t,
            count: _
        })) if t == topic
    ));
}

#[test]
fn try_recv_returns_closed_after_topic_removed() {
    let mw = Middleware::new();
    let topic = common::unique_topic("closed_try_recv");

    let mut sub = mw.subscribe::<String>(&topic).unwrap();
    assert!(mw.remove_topic(&topic));

    let closed = common::wait_until(
        Duration::from_millis(100),
        Duration::from_millis(5),
        || {
            matches!(
                sub.try_recv(),
                Some(Err(MiddlewareError::Closed { topic: t })) if t == topic
            )
        },
    );

    assert!(closed, "expected Closed after removing topic");
}

#[tokio::test]
async fn raw_bytes_sender_and_subscriber() {
    let mw = Middleware::new();
    let topic = common::unique_topic("raw_bytes");

    let mut rx = mw.subscribe_raw(&topic, Qos::KeepLast(8)).unwrap();
    let tx = mw.get_or_create_sender_raw(&topic, Qos::KeepLast(8)).unwrap();

    let payload = Arc::new(vec![1_u8, 2, 3, 4]);
    let sent = tx.send(payload.clone()).unwrap();
    assert_eq!(sent, 1);

    let received = rx.recv().await.unwrap();
    assert_eq!(&*received, &[1_u8, 2, 3, 4]);
    assert!(Arc::ptr_eq(&received, &payload));
}

#[test]
fn raw_bytes_type_mismatch_rejected() {
    let mw = Middleware::new();
    let topic = common::unique_topic("raw_conflict");

    let _typed = mw.subscribe::<String>(&topic).unwrap();

    assert!(mw.get_or_create_sender_raw(&topic, Qos::default()).is_none());
    assert!(mw.subscribe_raw(&topic, Qos::default()).is_none());
}
