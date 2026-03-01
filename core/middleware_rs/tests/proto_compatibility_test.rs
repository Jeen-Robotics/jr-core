//! Proto compatibility tests for jr-msgs generated Rust types
//!
//! These tests verify that the prost-generated Rust types are compatible
//! with the C++ protobuf wire format from jr-msgs.

use middleware_rs::proto::{std_msgs, geometry_msgs, sensor_msgs, jr};
use prost::Message;

// ============================================================================
// std_msgs tests
// ============================================================================

#[test]
fn test_header_roundtrip() {
    let header = std_msgs::Header {
        seq: 42,
        stamp: Some(std_msgs::Time {
            sec: 1234,
            nsec: 567890,
        }),
        frame_id: "base_link".to_string(),
    };

    // Encode
    let mut buf = Vec::new();
    header.encode(&mut buf).unwrap();

    // Decode
    let decoded = std_msgs::Header::decode(buf.as_slice()).unwrap();

    assert_eq!(decoded.seq, 42);
    assert_eq!(decoded.stamp.as_ref().unwrap().sec, 1234);
    assert_eq!(decoded.stamp.as_ref().unwrap().nsec, 567890);
    assert_eq!(decoded.frame_id, "base_link");
}

#[test]
fn test_time_roundtrip() {
    let time = std_msgs::Time {
        sec: 1709297000,
        nsec: 123456789,
    };

    let mut buf = Vec::new();
    time.encode(&mut buf).unwrap();
    let decoded = std_msgs::Time::decode(buf.as_slice()).unwrap();

    assert_eq!(decoded.sec, time.sec);
    assert_eq!(decoded.nsec, time.nsec);
}

// ============================================================================
// geometry_msgs tests
// ============================================================================

#[test]
fn test_vector3_roundtrip() {
    let vec = geometry_msgs::Vector3 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };

    let mut buf = Vec::new();
    vec.encode(&mut buf).unwrap();
    let decoded = geometry_msgs::Vector3::decode(buf.as_slice()).unwrap();

    assert!((decoded.x - 1.0).abs() < f64::EPSILON);
    assert!((decoded.y - 2.0).abs() < f64::EPSILON);
    assert!((decoded.z - 3.0).abs() < f64::EPSILON);
}

#[test]
fn test_quaternion_roundtrip() {
    let quat = geometry_msgs::Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.707,
        w: 0.707,
    };

    let mut buf = Vec::new();
    quat.encode(&mut buf).unwrap();
    let decoded = geometry_msgs::Quaternion::decode(buf.as_slice()).unwrap();

    assert!((decoded.x - 0.0).abs() < f64::EPSILON);
    assert!((decoded.y - 0.0).abs() < f64::EPSILON);
    assert!((decoded.z - 0.707).abs() < 0.001);
    assert!((decoded.w - 0.707).abs() < 0.001);
}

#[test]
fn test_transform_roundtrip() {
    let transform = geometry_msgs::Transform {
        translation: Some(geometry_msgs::Vector3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        }),
        rotation: Some(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }),
    };

    let mut buf = Vec::new();
    transform.encode(&mut buf).unwrap();
    let decoded = geometry_msgs::Transform::decode(buf.as_slice()).unwrap();

    assert!(decoded.translation.is_some());
    assert!(decoded.rotation.is_some());
    assert!((decoded.translation.as_ref().unwrap().x - 1.0).abs() < f64::EPSILON);
}

// ============================================================================
// sensor_msgs tests
// ============================================================================

#[test]
fn test_image_roundtrip() {
    let image = sensor_msgs::Image {
        header: Some(std_msgs::Header {
            seq: 1,
            stamp: Some(std_msgs::Time { sec: 100, nsec: 0 }),
            frame_id: "camera".to_string(),
        }),
        height: 480,
        width: 640,
        encoding: "rgb8".to_string(),
        is_bigendian: false,
        step: 640 * 3,
        data: vec![0u8; 640 * 480 * 3],
    };

    let mut buf = Vec::new();
    image.encode(&mut buf).unwrap();
    let decoded = sensor_msgs::Image::decode(buf.as_slice()).unwrap();

    assert_eq!(decoded.height, 480);
    assert_eq!(decoded.width, 640);
    assert_eq!(decoded.encoding, "rgb8");
    assert_eq!(decoded.data.len(), 640 * 480 * 3);
}

#[test]
fn test_imu_roundtrip() {
    let imu = sensor_msgs::Imu {
        header: Some(std_msgs::Header {
            seq: 100,
            stamp: Some(std_msgs::Time { sec: 1000, nsec: 500000 }),
            frame_id: "imu_link".to_string(),
        }),
        orientation: Some(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }),
        orientation_covariance: vec![0.0; 9],
        angular_velocity: Some(geometry_msgs::Vector3 {
            x: 0.01,
            y: 0.02,
            z: 0.03,
        }),
        angular_velocity_covariance: vec![0.0; 9],
        linear_acceleration: Some(geometry_msgs::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 9.81,
        }),
        linear_acceleration_covariance: vec![0.0; 9],
    };

    let mut buf = Vec::new();
    imu.encode(&mut buf).unwrap();
    let decoded = sensor_msgs::Imu::decode(buf.as_slice()).unwrap();

    assert!(decoded.orientation.is_some());
    assert!(decoded.angular_velocity.is_some());
    assert!(decoded.linear_acceleration.is_some());
    assert!((decoded.linear_acceleration.as_ref().unwrap().z - 9.81).abs() < 0.01);
}

#[test]
fn test_camera_info_roundtrip() {
    let info = sensor_msgs::CameraInfo {
        header: Some(std_msgs::Header {
            seq: 1,
            stamp: Some(std_msgs::Time { sec: 100, nsec: 0 }),
            frame_id: "camera_optical".to_string(),
        }),
        height: 480,
        width: 640,
        distortion_model: "plumb_bob".to_string(),
        d: vec![0.0; 5],
        k: vec![500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
        r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p: vec![500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x: 1,
        binning_y: 1,
        roi: Some(sensor_msgs::RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height: 480,
            width: 640,
            do_rectify: false,
        }),
    };

    let mut buf = Vec::new();
    info.encode(&mut buf).unwrap();
    let decoded = sensor_msgs::CameraInfo::decode(buf.as_slice()).unwrap();

    assert_eq!(decoded.height, 480);
    assert_eq!(decoded.width, 640);
    assert_eq!(decoded.k.len(), 9);
    assert!(decoded.roi.is_some());
}

// ============================================================================
// bag (middleware-specific) tests
// ============================================================================

#[test]
fn test_bag_header_roundtrip() {
    let header = jr::mw::BagHeader {
        version: 1,
        format_id: "JR_BAG".to_string(),
        compression: jr::mw::CompressionType::CompressionZstd as i32,
    };

    let mut buf = Vec::new();
    header.encode(&mut buf).unwrap();
    let decoded = jr::mw::BagHeader::decode(buf.as_slice()).unwrap();

    assert_eq!(decoded.version, 1);
    assert_eq!(decoded.format_id, "JR_BAG");
    assert_eq!(decoded.compression, jr::mw::CompressionType::CompressionZstd as i32);
}

#[test]
fn test_bag_record_roundtrip() {
    let record = jr::mw::BagRecord {
        topic: "/camera/image".to_string(),
        type_full_name: "sensor_msgs.Image".to_string(),
        payload: vec![1, 2, 3, 4, 5],
        timestamp_ns: 1234567890123456789,
    };

    let mut buf = Vec::new();
    record.encode(&mut buf).unwrap();
    let decoded = jr::mw::BagRecord::decode(buf.as_slice()).unwrap();

    assert_eq!(decoded.topic, "/camera/image");
    assert_eq!(decoded.type_full_name, "sensor_msgs.Image");
    assert_eq!(decoded.payload, vec![1, 2, 3, 4, 5]);
    assert_eq!(decoded.timestamp_ns, 1234567890123456789);
}

#[test]
fn test_video_frame_reference_roundtrip() {
    let frame_ref = jr::mw::VideoFrameReference {
        topic: "/camera/image".to_string(),
        video_file: "recording_camera.mp4".to_string(),
        frame_index: 42,
        width: 1920,
        height: 1080,
        encoding: "8UC3".to_string(),
        seq: 100,
    };

    let mut buf = Vec::new();
    frame_ref.encode(&mut buf).unwrap();
    let decoded = jr::mw::VideoFrameReference::decode(buf.as_slice()).unwrap();

    assert_eq!(decoded.topic, "/camera/image");
    assert_eq!(decoded.video_file, "recording_camera.mp4");
    assert_eq!(decoded.frame_index, 42);
    assert_eq!(decoded.width, 1920);
    assert_eq!(decoded.height, 1080);
}

// ============================================================================
// Wire format / encoded size tests
// ============================================================================

#[test]
fn test_encoded_size() {
    let header = std_msgs::Header {
        seq: 42,
        stamp: Some(std_msgs::Time { sec: 100, nsec: 200 }),
        frame_id: "test".to_string(),
    };

    let encoded_len = header.encoded_len();
    let mut buf = Vec::new();
    header.encode(&mut buf).unwrap();

    assert_eq!(buf.len(), encoded_len);
    assert!(encoded_len > 0);
}

#[test]
fn test_empty_image() {
    let image = sensor_msgs::Image {
        header: None,
        height: 0,
        width: 0,
        encoding: String::new(),
        is_bigendian: false,
        step: 0,
        data: vec![],
    };

    let mut buf = Vec::new();
    image.encode(&mut buf).unwrap();
    
    // Empty message should still encode
    let decoded = sensor_msgs::Image::decode(buf.as_slice()).unwrap();
    assert_eq!(decoded.height, 0);
    assert_eq!(decoded.data.len(), 0);
}
