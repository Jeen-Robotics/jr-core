mod common;

use std::sync::Arc;

use middleware::{Qos, TopicRegistry};

#[test]
fn create_topic() {
    let registry = TopicRegistry::new();
    let sender = registry.get_or_create_sender::<String>("test_topic");
    assert!(sender.is_some());
    assert!(registry.has_topic("test_topic"));
}

#[test]
fn type_mismatch() {
    let registry = TopicRegistry::new();

    let _ = registry.get_or_create_sender::<String>("typed_topic");
    let result = registry.get_or_create_sender::<i32>("typed_topic");
    assert!(result.is_none());
}

#[test]
fn subscribe_creates_topic() {
    let registry = TopicRegistry::new();
    let topic = common::unique_topic("new_topic");

    let rx = registry.subscribe::<String>(&topic);
    assert!(rx.is_some());
    assert!(registry.has_topic(&topic));
}

#[test]
fn publish_no_subscribers() {
    let registry = TopicRegistry::new();
    let topic = common::unique_topic("empty_topic");

    let count = registry.publish(&topic, Arc::new("hello".to_string()));
    assert_eq!(count, Some(0));
}

#[test]
fn remove_topic() {
    let registry = TopicRegistry::new();
    let topic = common::unique_topic("removable_topic");

    registry.get_or_create_sender::<String>(&topic);
    assert!(registry.has_topic(&topic));
    assert!(registry.remove_topic(&topic));
    assert!(!registry.has_topic(&topic));
}

#[test]
fn qos_sensor_data() {
    let registry = TopicRegistry::new();
    let topic = common::unique_topic("sensor_topic");

    let _rx = registry.subscribe_with_qos::<f64>(&topic, Qos::SensorData);

    assert!(registry.has_topic(&topic));
    assert_eq!(registry.get_qos(&topic), Some(Qos::SensorData));
}

#[test]
fn qos_keep_last() {
    let registry = TopicRegistry::new();
    let topic = common::unique_topic("logs_topic");

    let _rx = registry.subscribe_with_qos::<String>(&topic, Qos::KeepLast(1024));

    assert_eq!(registry.get_qos(&topic), Some(Qos::KeepLast(1024)));
}

#[test]
fn subscriber_count_and_topic_names() {
    let registry = TopicRegistry::new();
    let topic = common::unique_topic("count_topic");

    let _rx1 = registry.subscribe::<String>(&topic).unwrap();
    let _rx2 = registry.subscribe::<String>(&topic).unwrap();

    assert_eq!(registry.subscriber_count::<String>(&topic), Some(2));
    assert_eq!(registry.subscriber_count::<i32>(&topic), None);

    let names = registry.topic_names();
    assert!(names.contains(&topic));
}
