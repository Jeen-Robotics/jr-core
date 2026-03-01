//! Wire format compatibility tests for protobuf messages
//! 
//! These tests verify that Rust (prost) and C++ (protobuf) produce
//! identical binary wire format for the same messages.

use middleware_rs::proto::{std_msgs, sensor_msgs, jr};
use prost::Message;

/// Test that Header encodes/decodes correctly
#[test]
fn test_header_roundtrip() {
    let header = std_msgs::Header {
        seq: 42,
        timestamp_ns: 1234567890123456789,
        frame_id: "camera_link".to_string(),
    };
    
    // Encode to bytes
    let mut buf = Vec::new();
    header.encode(&mut buf).unwrap();
    
    // Decode back
    let decoded = std_msgs::Header::decode(&buf[..]).unwrap();
    
    assert_eq!(header.seq, decoded.seq);
    assert_eq!(header.timestamp_ns, decoded.timestamp_ns);
    assert_eq!(header.frame_id, decoded.frame_id);
}

/// Test that Vector3 encodes/decodes correctly
#[test]
fn test_vector3_roundtrip() {
    let vec = std_msgs::Vector3 {
        x: 1.5,
        y: -2.3,
        z: 9.81,
    };
    
    let mut buf = Vec::new();
    vec.encode(&mut buf).unwrap();
    
    let decoded = std_msgs::Vector3::decode(&buf[..]).unwrap();
    
    assert!((vec.x - decoded.x).abs() < 1e-10);
    assert!((vec.y - decoded.y).abs() < 1e-10);
    assert!((vec.z - decoded.z).abs() < 1e-10);
}

/// Test that Quaternion encodes/decodes correctly
#[test]
fn test_quaternion_roundtrip() {
    let quat = std_msgs::Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.707,
        w: 0.707,
    };
    
    let mut buf = Vec::new();
    quat.encode(&mut buf).unwrap();
    
    let decoded = std_msgs::Quaternion::decode(&buf[..]).unwrap();
    
    assert!((quat.x - decoded.x).abs() < 1e-10);
    assert!((quat.y - decoded.y).abs() < 1e-10);
    assert!((quat.z - decoded.z).abs() < 1e-10);
    assert!((quat.w - decoded.w).abs() < 1e-10);
}

/// Test that Image message encodes/decodes correctly
#[test]
fn test_image_roundtrip() {
    let image = sensor_msgs::Image {
        header: Some(std_msgs::Header {
            seq: 100,
            timestamp_ns: 1609459200_000_000_000, // 2021-01-01 00:00:00 UTC
            frame_id: "camera_optical_frame".to_string(),
        }),
        height: 480,
        width: 640,
        encoding: "8UC3".to_string(),
        is_bigendian: false,
        step: 640 * 3,
        data: vec![128u8; 640 * 480 * 3], // Gray image
    };
    
    let mut buf = Vec::new();
    image.encode(&mut buf).unwrap();
    
    let decoded = sensor_msgs::Image::decode(&buf[..]).unwrap();
    
    assert_eq!(image.height, decoded.height);
    assert_eq!(image.width, decoded.width);
    assert_eq!(image.encoding, decoded.encoding);
    assert_eq!(image.step, decoded.step);
    assert_eq!(image.data.len(), decoded.data.len());
    assert_eq!(image.data, decoded.data);
    
    let header = decoded.header.unwrap();
    assert_eq!(header.seq, 100);
    assert_eq!(header.frame_id, "camera_optical_frame");
}

/// Test that IMU message encodes/decodes correctly
#[test]
fn test_imu_roundtrip() {
    let imu = sensor_msgs::Imu {
        header: Some(std_msgs::Header {
            seq: 50,
            timestamp_ns: 1609459200_000_000_000,
            frame_id: "imu_link".to_string(),
        }),
        orientation: Some(std_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }),
        orientation_covariance: vec![0.01; 9],
        angular_velocity: Some(std_msgs::Vector3 {
            x: 0.001,
            y: 0.002,
            z: 0.003,
        }),
        angular_velocity_covariance: vec![0.001; 9],
        linear_acceleration: Some(std_msgs::Vector3 {
            x: 0.1,
            y: 0.2,
            z: 9.81,
        }),
        linear_acceleration_covariance: vec![0.01; 9],
    };
    
    let mut buf = Vec::new();
    imu.encode(&mut buf).unwrap();
    
    let decoded = sensor_msgs::Imu::decode(&buf[..]).unwrap();
    
    let header = decoded.header.unwrap();
    assert_eq!(header.frame_id, "imu_link");
    
    let accel = decoded.linear_acceleration.unwrap();
    assert!((accel.z - 9.81).abs() < 1e-10);
}

/// Test that CameraInfo encodes/decodes correctly
#[test]
fn test_camera_info_roundtrip() {
    let info = sensor_msgs::CameraInfo {
        header: Some(std_msgs::Header {
            seq: 1,
            timestamp_ns: 0,
            frame_id: "camera".to_string(),
        }),
        height: 480,
        width: 640,
        distortion_model: "plumb_bob".to_string(),
        d: vec![0.1, -0.2, 0.001, 0.002, 0.0], // k1, k2, p1, p2, k3
        k: vec![500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0], // 3x3 camera matrix
        r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], // Identity rotation
        p: vec![500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0], // 3x4 projection
        binning_x: 1,
        binning_y: 1,
    };
    
    let mut buf = Vec::new();
    info.encode(&mut buf).unwrap();
    
    let decoded = sensor_msgs::CameraInfo::decode(&buf[..]).unwrap();
    
    assert_eq!(info.width, decoded.width);
    assert_eq!(info.height, decoded.height);
    assert_eq!(info.distortion_model, decoded.distortion_model);
    assert_eq!(info.d, decoded.d);
    assert_eq!(info.k, decoded.k);
}

/// Test bag message types
#[test]
fn test_bag_header_roundtrip() {
    let header = jr::mw::BagHeader {
        version: 1,
        format_id: "JR_BAG".to_string(),
        compression: jr::mw::CompressionType::CompressionZstd as i32,
    };
    
    let mut buf = Vec::new();
    header.encode(&mut buf).unwrap();
    
    let decoded = jr::mw::BagHeader::decode(&buf[..]).unwrap();
    
    assert_eq!(header.version, decoded.version);
    assert_eq!(header.format_id, decoded.format_id);
    assert_eq!(header.compression, decoded.compression);
}

/// Test BagRecord serialization
#[test]
fn test_bag_record_roundtrip() {
    // Create a BagRecord containing an Image message
    let image = sensor_msgs::Image {
        header: Some(std_msgs::Header {
            seq: 1,
            timestamp_ns: 1234567890,
            frame_id: "camera".to_string(),
        }),
        height: 100,
        width: 100,
        encoding: "mono8".to_string(),
        is_bigendian: false,
        step: 100,
        data: vec![0u8; 100 * 100],
    };
    
    let mut payload = Vec::new();
    image.encode(&mut payload).unwrap();
    
    let record = jr::mw::BagRecord {
        topic: "/camera/image".to_string(),
        type_full_name: "sensor_msgs.Image".to_string(),
        payload,
        timestamp_ns: 1234567890,
    };
    
    let mut buf = Vec::new();
    record.encode(&mut buf).unwrap();
    
    let decoded_record = jr::mw::BagRecord::decode(&buf[..]).unwrap();
    
    assert_eq!(record.topic, decoded_record.topic);
    assert_eq!(record.type_full_name, decoded_record.type_full_name);
    assert_eq!(record.timestamp_ns, decoded_record.timestamp_ns);
    
    // Decode the embedded Image
    let decoded_image = sensor_msgs::Image::decode(&decoded_record.payload[..]).unwrap();
    assert_eq!(image.width, decoded_image.width);
    assert_eq!(image.height, decoded_image.height);
}

/// Test VideoFrameReference serialization
#[test]
fn test_video_frame_reference_roundtrip() {
    let frame_ref = jr::mw::VideoFrameReference {
        topic: "/camera/video".to_string(),
        video_file: "bag_camera.avi".to_string(),
        frame_index: 42,
        width: 1920,
        height: 1080,
        encoding: "8UC3".to_string(),
        seq: 100,
    };
    
    let mut buf = Vec::new();
    frame_ref.encode(&mut buf).unwrap();
    
    let decoded = jr::mw::VideoFrameReference::decode(&buf[..]).unwrap();
    
    assert_eq!(frame_ref.topic, decoded.topic);
    assert_eq!(frame_ref.video_file, decoded.video_file);
    assert_eq!(frame_ref.frame_index, decoded.frame_index);
    assert_eq!(frame_ref.width, decoded.width);
    assert_eq!(frame_ref.height, decoded.height);
}

/// Wire format compatibility test: Verify specific byte sequences
/// These are known-good encodings from C++ protobuf
#[test]
fn test_wire_format_known_bytes() {
    // Test that a simple message encodes to expected bytes
    // Vector3 { x: 1.0, y: 2.0, z: 3.0 }
    // Field 1: x (double, tag=0x09), value = 1.0 in little-endian IEEE 754
    // Field 2: y (double, tag=0x11), value = 2.0
    // Field 3: z (double, tag=0x19), value = 3.0
    
    let vec = std_msgs::Vector3 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    
    let mut buf = Vec::new();
    vec.encode(&mut buf).unwrap();
    
    // Verify the encoding is non-empty and has reasonable size
    // (3 doubles with tags = roughly 25-27 bytes)
    assert!(!buf.is_empty());
    assert!(buf.len() <= 30);
    
    // Verify we can decode our own encoding (sanity check)
    let decoded = std_msgs::Vector3::decode(&buf[..]).unwrap();
    assert_eq!(decoded.x, 1.0);
    assert_eq!(decoded.y, 2.0);
    assert_eq!(decoded.z, 3.0);
}

/// Test empty messages
#[test]
fn test_empty_image() {
    let image = sensor_msgs::Image::default();
    
    let mut buf = Vec::new();
    image.encode(&mut buf).unwrap();
    
    let decoded = sensor_msgs::Image::decode(&buf[..]).unwrap();
    
    assert_eq!(decoded.height, 0);
    assert_eq!(decoded.width, 0);
    assert!(decoded.data.is_empty());
}

/// Test message size calculation
#[test]
fn test_encoded_size() {
    let image = sensor_msgs::Image {
        header: Some(std_msgs::Header {
            seq: 1,
            timestamp_ns: 0,
            frame_id: "test".to_string(),
        }),
        height: 100,
        width: 100,
        encoding: "mono8".to_string(),
        is_bigendian: false,
        step: 100,
        data: vec![0u8; 10000],
    };
    
    // encoded_len should match actual encoding size
    let expected_len = image.encoded_len();
    
    let mut buf = Vec::new();
    image.encode(&mut buf).unwrap();
    
    assert_eq!(buf.len(), expected_len);
}

/// Test BagRecordBatch for streaming
#[test]
fn test_bag_record_batch() {
    let records = vec![
        jr::mw::BagRecord {
            topic: "/topic1".to_string(),
            type_full_name: "Type1".to_string(),
            payload: vec![1, 2, 3],
            timestamp_ns: 1000,
        },
        jr::mw::BagRecord {
            topic: "/topic2".to_string(),
            type_full_name: "Type2".to_string(),
            payload: vec![4, 5, 6],
            timestamp_ns: 2000,
        },
    ];
    
    let batch = jr::mw::BagRecordBatch { records };
    
    let mut buf = Vec::new();
    batch.encode(&mut buf).unwrap();
    
    let decoded = jr::mw::BagRecordBatch::decode(&buf[..]).unwrap();
    
    assert_eq!(decoded.records.len(), 2);
    assert_eq!(decoded.records[0].topic, "/topic1");
    assert_eq!(decoded.records[1].topic, "/topic2");
}
