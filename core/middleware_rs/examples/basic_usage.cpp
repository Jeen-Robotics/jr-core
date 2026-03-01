/// @file basic_usage.cpp
/// @brief Example demonstrating middleware C++ API usage
///
/// Build with CMake after enabling BUILD_RUST_MIDDLEWARE:
///   cmake -DBUILD_RUST_MIDDLEWARE=ON ..
///   make middleware_rs_example

#include <middleware_rs/middleware.hpp>

// Include protobuf messages from jr_msgs
#include <std_msgs.pb.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

int main() {
    // Initialize the middleware (idempotent, safe to call multiple times)
    jr::mw::init();
    
    std::atomic<bool> running{true};
    
    // =========================================================================
    // Example 1: Basic Publish/Subscribe with Callback
    // =========================================================================
    
    std::cout << "=== Example 1: Callback-based subscription ===" << std::endl;
    
    // Create a subscriber with callback
    auto sub = jr::mw::subscribe<std_msgs::Int32>(
        "example/counter",
        [](const std_msgs::Int32& msg) {
            std::cout << "Received: " << msg.data() << std::endl;
        }
    );
    
    // Create a publisher
    auto pub = jr::mw::advertise<std_msgs::Int32>("example/counter");
    
    if (!pub) {
        std::cerr << "Failed to create publisher" << std::endl;
        return 1;
    }
    
    // Publish some messages
    for (int i = 0; i < 5; ++i) {
        std_msgs::Int32 msg;
        msg.set_data(i);
        pub.publish(msg);
        
        // Process callbacks
        sub.spin_once();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // =========================================================================
    // Example 2: Polling-based Subscription
    // =========================================================================
    
    std::cout << "\n=== Example 2: Polling-based subscription ===" << std::endl;
    
    auto poll_sub = jr::mw::subscribe<std_msgs::Int32>(
        "example/poll_topic"
    );
    
    auto poll_pub = jr::mw::advertise<std_msgs::Int32>("example/poll_topic");
    
    // Publish a message
    std_msgs::Int32 poll_msg;
    poll_msg.set_data(42);
    poll_pub.publish(poll_msg);
    
    // Wait for message to be available
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Poll for messages
    if (auto msg = poll_sub.try_recv()) {
        std::cout << "Polled message: " << msg->data() << std::endl;
    }
    
    // =========================================================================
    // Example 3: SensorData QoS (Realtime)
    // =========================================================================
    
    std::cout << "\n=== Example 3: SensorData QoS ===" << std::endl;
    
    auto sensor_sub = jr::mw::subscribe<std_msgs::Int32>(
        "example/sensor",
        jr::mw::Qos::SensorData
    );
    
    auto sensor_pub = jr::mw::advertise<std_msgs::Int32>(
        "example/sensor",
        jr::mw::Qos::SensorData
    );
    
    // Rapid publishing - some messages may be dropped (that's expected with SensorData)
    for (int i = 0; i < 100; ++i) {
        std_msgs::Int32 msg;
        msg.set_data(i);
        sensor_pub.publish(msg);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Should get some of the recent messages
    int received = 0;
    while (auto msg = sensor_sub.try_recv()) {
        std::cout << "Sensor value: " << msg->data() << std::endl;
        ++received;
    }
    std::cout << "Received " << received << " sensor messages" << std::endl;
    
    // =========================================================================
    // Example 4: Using eventfd for Efficient Waiting (Linux)
    // =========================================================================
    
#ifdef __linux__
    std::cout << "\n=== Example 4: eventfd-based waiting ===" << std::endl;
    
    auto wait_sub = jr::mw::subscribe<std_msgs::Int32>("example/wait");
    auto wait_pub = jr::mw::advertise<std_msgs::Int32>("example/wait");
    
    // Check we have a valid fd
    int fd = wait_sub.get_fd();
    if (fd >= 0) {
        std::cout << "Got eventfd: " << fd << std::endl;
        
        // Start a thread that publishes after a delay
        std::thread publisher([&wait_pub]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std_msgs::Int32 msg;
            msg.set_data(999);
            wait_pub.publish(msg);
        });
        
        // Wait for message using poll()
        if (wait_sub.wait(std::chrono::milliseconds(500))) {
            if (auto msg = wait_sub.try_recv()) {
                std::cout << "Received after wait: " << msg->data() << std::endl;
            }
        }
        
        publisher.join();
    }
#endif
    
    std::cout << "\nExample complete!" << std::endl;
    return 0;
}
