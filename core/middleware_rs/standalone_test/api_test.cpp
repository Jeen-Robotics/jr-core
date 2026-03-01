/// @file api_test.cpp
/// @brief Simple API compilation and runtime test
///
/// Tests the raw FFI wrapper without template instantiation to avoid
/// protobuf move/copy issues.

#include <middleware_rs/fwd.hpp>
#include <middleware_rs/middleware.hpp>

#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>

// Test the low-level FFI wrapper directly (no proto templates)
int main() {
    std::cout << "=== middleware_rs C++ API Test ===" << std::endl;
    
    int passed = 0;
    int failed = 0;
    
    auto test = [&](const char* name, bool result) {
        std::cout << (result ? "[PASS] " : "[FAIL] ") << name << std::endl;
        if (result) ++passed; else ++failed;
    };
    
    // Test 1: Initialize middleware
    bool init_ok = jr::mw::init();
    test("Initialize middleware", init_ok);
    
    // Test 2: Create publisher (using detail API)
    auto pub = jr::mw::detail::create_publisher_impl("test/raw", jr::mw::Qos::KeepLast, 16);
    test("Create publisher", jr::mw::detail::publisher_valid_impl(pub.get()));
    
    // Test 3: Create subscriber (using detail API)
    auto sub = jr::mw::detail::create_subscriber_impl("test/raw", jr::mw::Qos::KeepLast, 16);
    test("Create subscriber", jr::mw::detail::subscriber_valid_impl(sub.get()));
    
    // Test 4: Topic exists
    bool exists = jr::mw::topic_exists("test/raw");
    test("Topic exists", exists);
    
    // Test 5: Publish raw bytes
    const char* msg = "hello world";
    bool pub_ok = jr::mw::detail::publish_impl(pub.get(), msg, strlen(msg));
    test("Publish message", pub_ok);
    
    // Test 6: Receive raw bytes
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto result = jr::mw::detail::subscriber_recv_impl(sub.get());
    test("Receive message", result.has_message);
    
    // Test 7: Message content
    bool content_ok = result.has_message && 
        std::string(result.data.begin(), result.data.end()) == "hello world";
    test("Message content matches", content_ok);
    
    // Test 8: SensorData QoS
    auto sensor_pub = jr::mw::detail::create_publisher_impl("test/sensor", jr::mw::Qos::SensorData, 1);
    test("SensorData publisher", jr::mw::detail::publisher_valid_impl(sensor_pub.get()));
    
    auto sensor_sub = jr::mw::detail::create_subscriber_impl("test/sensor", jr::mw::Qos::SensorData, 1);
    test("SensorData subscriber", jr::mw::detail::subscriber_valid_impl(sensor_sub.get()));
    
    // Test 9: Rapid publish with SensorData (should not block)
    for (int i = 0; i < 100; ++i) {
        char buf[16];
        snprintf(buf, sizeof(buf), "msg%d", i);
        jr::mw::detail::publish_impl(sensor_pub.get(), buf, strlen(buf));
    }
    test("Rapid SensorData publish", true);
    
    // Test 10: Receive some messages (may drop old ones with SensorData QoS)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int received_count = 0;
    while (true) {
        auto r = jr::mw::detail::subscriber_recv_impl(sensor_sub.get());
        if (!r.has_message) break;
        ++received_count;
    }
    test("SensorData received some messages", received_count > 0);
    std::cout << "  (received " << received_count << " of 100)" << std::endl;
    
    // Summary
    std::cout << "\n=== Results: " << passed << "/" << (passed + failed) 
              << " passed ===" << std::endl;
    
    return failed > 0 ? 1 : 0;
}
