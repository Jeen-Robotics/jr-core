mod common;

use std::sync::Arc;

use middleware::Middleware;
use tokio::time::{timeout, Duration};

#[tokio::test]
async fn full_pubsub_workflow() {
    let mw = Middleware::new();
    let topic = common::unique_topic("workflow");

    let mut sub1 = mw.subscribe::<String>(&topic).unwrap();
    let mut sub2 = mw.subscribe::<String>(&topic).unwrap();

    let message = Arc::new("workflow test message".to_string());
    let receiver_count = mw.publish(&topic, message.clone()).unwrap();
    assert_eq!(receiver_count, 2);

    let r1 = sub1.recv().await.unwrap();
    let r2 = sub2.recv().await.unwrap();

    assert_eq!(*r1, "workflow test message");
    assert_eq!(*r2, "workflow test message");
    assert!(Arc::ptr_eq(&r1, &r2));
}

#[derive(Debug, Clone, PartialEq)]
struct CameraFrame {
    width: u32,
    height: u32,
    data: Vec<u8>,
    timestamp_ns: u64,
}

#[tokio::test]
async fn complex_message_types() {
    let mw = Middleware::new();
    let topic = common::unique_topic("camera_front");

    let mut sub = mw.subscribe::<CameraFrame>(&topic).unwrap();

    let frame = CameraFrame {
        width: 1920,
        height: 1080,
        data: vec![0_u8; 1920 * 1080 * 3],
        timestamp_ns: 1_234_567_890,
    };

    let frame_arc = Arc::new(frame);
    let original_ptr = Arc::as_ptr(&frame_arc);

    mw.publish(&topic, frame_arc).unwrap();

    let received = sub.recv().await.unwrap();

    assert_eq!(Arc::as_ptr(&received), original_ptr);
    assert_eq!(received.width, 1920);
    assert_eq!(received.height, 1080);
    assert_eq!(received.data.len(), 1920 * 1080 * 3);
}

#[tokio::test]
async fn concurrent_pubsub() {
    let mw = Arc::new(Middleware::new());
    let topic = common::unique_topic("concurrent");
    let message_count = 100;

    let subscriptions: Vec<_> = (0..3).map(|_| mw.subscribe::<i32>(&topic).unwrap()).collect();

    let handles: Vec<_> = subscriptions
        .into_iter()
        .enumerate()
        .map(|(sub_id, mut sub)| {
            tokio::spawn(async move {
                let mut received = vec![];
                for _ in 0..message_count {
                    match timeout(Duration::from_secs(5), sub.recv()).await {
                        Ok(Ok(msg)) => received.push(*msg),
                        Ok(Err(e)) => panic!("Subscriber {} error: {:?}", sub_id, e),
                        Err(_) => panic!("Subscriber {} timeout", sub_id),
                    }
                }
                received
            })
        })
        .collect();

    for i in 0..message_count {
        mw.publish_owned(&topic, i).unwrap();
    }

    for handle in handles {
        let received = handle.await.unwrap();
        assert_eq!(received.len(), message_count as usize);
        for (i, &msg) in received.iter().enumerate() {
            assert_eq!(msg, i as i32);
        }
    }
}

#[tokio::test]
async fn unsubscribe_behavior() {
    let mw = Middleware::new();
    let topic = common::unique_topic("unsub");

    let mut sub1 = mw.subscribe::<String>(&topic).unwrap();
    let sub2 = mw.subscribe::<String>(&topic).unwrap();

    assert_eq!(mw.subscriber_count::<String>(&topic), Some(2));

    drop(sub2);
    assert_eq!(mw.subscriber_count::<String>(&topic), Some(1));

    mw.publish_owned(&topic, "still here".to_string()).unwrap();

    let msg = sub1.recv().await.unwrap();
    assert_eq!(*msg, "still here");

    drop(sub1);
    assert_eq!(mw.subscriber_count::<String>(&topic), Some(0));

    let count = mw.publish_owned(&topic, "nobody listening".to_string()).unwrap();
    assert_eq!(count, 0);
}

#[tokio::test]
async fn multiple_independent_topics() {
    let mw = Middleware::new();
    let camera_topic = common::unique_topic("camera");
    let imu_topic = common::unique_topic("imu");
    let gps_topic = common::unique_topic("gps");

    let mut cam_sub = mw.subscribe::<Vec<u8>>(&camera_topic).unwrap();
    let mut imu_sub = mw.subscribe::<(f64, f64, f64)>(&imu_topic).unwrap();
    let mut gps_sub = mw.subscribe::<(f64, f64)>(&gps_topic).unwrap();

    mw.publish_owned(&camera_topic, vec![1_u8, 2_u8, 3_u8]).unwrap();
    mw.publish_owned(&imu_topic, (1.0, 2.0, 3.0)).unwrap();
    mw.publish_owned(&gps_topic, (55.7558, 37.6173)).unwrap();

    let cam = cam_sub.recv().await.unwrap();
    let imu = imu_sub.recv().await.unwrap();
    let gps = gps_sub.recv().await.unwrap();

    assert_eq!(*cam, vec![1_u8, 2_u8, 3_u8]);
    assert_eq!(*imu, (1.0, 2.0, 3.0));
    assert_eq!(*gps, (55.7558, 37.6173));
}

#[tokio::test]
async fn zero_copy_arc_verification() {
    let mw = Middleware::new();
    let topic = common::unique_topic("arc_test");

    let mut sub1 = mw.subscribe::<String>(&topic).unwrap();
    let mut sub2 = mw.subscribe::<String>(&topic).unwrap();

    let msg = Arc::new("shared data".to_string());
    let original_ptr = Arc::as_ptr(&msg);

    mw.publish(&topic, msg.clone()).unwrap();

    let r1 = sub1.recv().await.unwrap();
    let r2 = sub2.recv().await.unwrap();

    assert_eq!(Arc::as_ptr(&r1), original_ptr);
    assert_eq!(Arc::as_ptr(&r2), original_ptr);
    assert!(Arc::ptr_eq(&r1, &r2));
    assert!(Arc::ptr_eq(&r1, &msg));
}
