#![cfg(feature = "test-hooks")]

mod common;

use std::time::Duration;

use middleware::ffi::{test_hooks, QosKind};

#[test]
fn middleware_init_is_idempotent() {
    assert!(test_hooks::middleware_init_for_tests());
    assert!(test_hooks::middleware_init_for_tests());
}

#[test]
fn publish_subscribe_eventfd_or_polling() {
    test_hooks::middleware_init_for_tests();

    let topic = common::unique_topic("eventfd_test");
    let mut sub = test_hooks::create_subscriber_for_tests(&topic, QosKind::KeepLast, 16);
    assert!(test_hooks::subscriber_is_valid_for_tests(&sub));

    let pub_ = test_hooks::create_publisher_for_tests(&topic, QosKind::KeepLast, 16);
    assert!(test_hooks::publisher_is_valid_for_tests(&pub_));

    #[cfg(target_os = "linux")]
    assert!(test_hooks::subscriber_get_fd_for_tests(&sub) >= 0);

    #[cfg(not(target_os = "linux"))]
    assert_eq!(test_hooks::subscriber_get_fd_for_tests(&sub), -1);

    let initial = test_hooks::subscriber_try_recv_for_tests(&mut sub);
    assert!(!initial.has_message);

    let publish_result = test_hooks::publish_bytes_for_tests(&pub_, b"hello eventfd");
    assert!(publish_result.success);

    let received = common::wait_until(
        Duration::from_millis(250),
        Duration::from_millis(5),
        || {
            let result = test_hooks::subscriber_try_recv_for_tests(&mut sub);
            result.has_message && result.data == b"hello eventfd"
        },
    );
    assert!(received, "did not receive published bytes");
}

#[test]
fn sensor_data_qos_delivers_messages() {
    test_hooks::middleware_init_for_tests();

    let topic = common::unique_topic("sensor_eventfd");
    let mut sub = test_hooks::create_subscriber_for_tests(&topic, QosKind::SensorData, 1);
    assert!(test_hooks::subscriber_is_valid_for_tests(&sub));

    let pub_ = test_hooks::create_publisher_for_tests(&topic, QosKind::SensorData, 1);

    for i in 0..10_u8 {
        let _ = test_hooks::publish_bytes_for_tests(&pub_, &[i]);
    }

    let received = common::wait_until(
        Duration::from_millis(250),
        Duration::from_millis(5),
        || test_hooks::subscriber_try_recv_for_tests(&mut sub).has_message,
    );

    assert!(received, "expected at least one sensor-data message");
}

#[test]
fn invalid_subscriber_and_publisher_from_type_conflict() {
    test_hooks::middleware_init_for_tests();

    let sub_topic = common::unique_topic("type_conflict_sub");
    test_hooks::force_string_topic_for_tests(&sub_topic);

    let mut sub = test_hooks::create_subscriber_for_tests(&sub_topic, QosKind::KeepLast, 16);
    assert!(!test_hooks::subscriber_is_valid_for_tests(&sub));
    assert_eq!(test_hooks::subscriber_get_fd_for_tests(&sub), -1);

    let sub_result = test_hooks::subscriber_try_recv_for_tests(&mut sub);
    assert!(!sub_result.has_message);
    assert!(sub_result.closed);

    let pub_topic = common::unique_topic("type_conflict_pub");
    test_hooks::force_string_topic_for_tests(&pub_topic);

    let pub_ = test_hooks::create_publisher_for_tests(&pub_topic, QosKind::KeepLast, 16);
    assert!(!test_hooks::publisher_is_valid_for_tests(&pub_));

    let pub_result = test_hooks::publish_bytes_for_tests(&pub_, b"test");
    assert!(!pub_result.success);
}

#[test]
fn topic_utilities_report_state() {
    test_hooks::middleware_init_for_tests();

    let topic = common::unique_topic("ffi_utils");
    assert!(!test_hooks::topic_exists_for_tests(&topic));

    let mut sub = test_hooks::create_subscriber_for_tests(&topic, QosKind::KeepLast, 8);
    assert!(test_hooks::subscriber_is_valid_for_tests(&sub));
    assert_eq!(test_hooks::subscriber_topic_for_tests(&sub), topic);

    let pub_ = test_hooks::create_publisher_for_tests(&topic, QosKind::KeepLast, 8);
    assert!(test_hooks::publisher_is_valid_for_tests(&pub_));

    let _ = test_hooks::publish_bytes_for_tests(&pub_, b"a");

    let count_is_non_zero = common::wait_until(
        Duration::from_millis(250),
        Duration::from_millis(5),
        || test_hooks::subscriber_count_for_tests(&topic) >= 1,
    );
    assert!(count_is_non_zero);
    assert!(test_hooks::topic_exists_for_tests(&topic));

    let topics = test_hooks::list_topics_for_tests();
    assert!(topics.contains(&topic));

    let _ = test_hooks::subscriber_try_recv_for_tests(&mut sub);
}

#[cfg(target_os = "linux")]
#[test]
fn epoll_integration() {
    test_hooks::middleware_init_for_tests();

    let topic = common::unique_topic("epoll_test");
    let mut sub = test_hooks::create_subscriber_for_tests(&topic, QosKind::KeepLast, 16);
    let pub_ = test_hooks::create_publisher_for_tests(&topic, QosKind::KeepLast, 16);

    let fd = test_hooks::subscriber_get_fd_for_tests(&sub);
    assert!(fd >= 0);

    let epfd = unsafe { libc::epoll_create1(0) };
    assert!(epfd >= 0);

    let mut ev = libc::epoll_event {
        events: libc::EPOLLIN as u32,
        u64: fd as u64,
    };
    let ret = unsafe { libc::epoll_ctl(epfd, libc::EPOLL_CTL_ADD, fd, &mut ev) };
    assert_eq!(ret, 0);

    let publish_result = test_hooks::publish_bytes_for_tests(&pub_, b"epoll test message");
    assert!(publish_result.success);

    let mut events = [libc::epoll_event { events: 0, u64: 0 }; 1];
    let n = unsafe { libc::epoll_wait(epfd, events.as_mut_ptr(), 1, 1000) };
    assert!(n > 0, "epoll_wait should return > 0");

    let result = test_hooks::subscriber_try_recv_for_tests(&mut sub);
    assert!(result.has_message);
    assert_eq!(result.data, b"epoll test message");

    unsafe {
        libc::close(epfd);
    }
}
