#include <gtest/gtest.h>

#include <middleware/detail/transport.hpp>
#include <middleware/middleware.hpp>
#include <middleware/node.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <google/protobuf/wrappers.pb.h>

#include "../test_common.hpp"

namespace jr::mw {

class MiddlewareIntegrationTest : public ::testing::Test {
protected:
  void TearDown() override {
    shutdown();
  }
};

TEST_F(MiddlewareIntegrationTest, PublishSubscribeDeliversLatest) {
  auto mw = Middleware::create();
  ASSERT_NE(mw, nullptr);

  const auto topic = test::unique_topic("/int");
  std::atomic<int> received{-1};

  auto sub = mw->subscribe<google::protobuf::Int32Value>(
    topic,
    [&](const google::protobuf::Int32Value& v) { received.store(v.value()); }
  );
  ASSERT_TRUE(sub.valid());

  google::protobuf::Int32Value v;
  v.set_value(1);
  mw->publish(topic, v);
  v.set_value(2);
  mw->publish(topic, v);
  v.set_value(3);
  mw->publish(topic, v);

  ASSERT_TRUE(test::wait_until(
    std::chrono::milliseconds(300),
    std::chrono::milliseconds(2),
    [&]() { return received.load() == 3; }
  ));
}

TEST_F(MiddlewareIntegrationTest, UnsubscribeStopsDelivery) {
  auto mw = Middleware::create();
  ASSERT_NE(mw, nullptr);

  const auto topic = test::unique_topic("/unsub");
  std::atomic<int> count{0};

  auto sub = mw->subscribe<google::protobuf::Int32Value>(
    topic,
    [&](const google::protobuf::Int32Value&) { ++count; }
  );
  ASSERT_TRUE(sub.valid());

  google::protobuf::Int32Value v;
  v.set_value(1);
  mw->publish(topic, v);

  ASSERT_TRUE(test::wait_until(
    std::chrono::milliseconds(300),
    std::chrono::milliseconds(2),
    [&]() { return count.load() >= 1; }
  ));

  sub.unsubscribe();
  const auto before = count.load();

  v.set_value(2);
  mw->publish(topic, v);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  EXPECT_EQ(count.load(), before);
}

TEST_F(MiddlewareIntegrationTest, TypeIsolationDifferentTypesSameTopic) {
  auto mw = Middleware::create();
  ASSERT_NE(mw, nullptr);

  const auto topic = test::unique_topic("/typed");
  std::atomic<int> ints{0};
  std::atomic<int> strings{0};

  auto int_sub = mw->subscribe<google::protobuf::Int32Value>(
    topic,
    [&](const google::protobuf::Int32Value&) { ++ints; }
  );
  ASSERT_TRUE(int_sub.valid());

  auto str_sub = mw->subscribe<google::protobuf::StringValue>(
    topic,
    [&](const google::protobuf::StringValue&) { ++strings; }
  );
  EXPECT_FALSE(str_sub.valid());

  google::protobuf::Int32Value int_msg;
  int_msg.set_value(9);
  mw->publish(topic, int_msg);

  ASSERT_TRUE(test::wait_until(
    std::chrono::milliseconds(300),
    std::chrono::milliseconds(2),
    [&]() { return ints.load() >= 1; }
  ));

  google::protobuf::StringValue str_msg;
  str_msg.set_value("hello");
  mw->publish(topic, str_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  EXPECT_EQ(strings.load(), 0);
}

TEST_F(MiddlewareIntegrationTest, PublishSerializedTypeMismatchDoesNotInvokeWrongSubscriber) {
  auto mw = Middleware::create();
  ASSERT_NE(mw, nullptr);

  const auto topic = test::unique_topic("/serialized");
  std::atomic<int> count{0};

  auto sub = mw->subscribe<google::protobuf::Int32Value>(
    topic,
    [&](const google::protobuf::Int32Value&) { ++count; }
  );
  ASSERT_TRUE(sub.valid());

  google::protobuf::Int32Value int_value;
  int_value.set_value(1);
  std::string int_payload;
  ASSERT_TRUE(int_value.SerializeToString(&int_payload));
  mw->publish_serialized(topic, "google.protobuf.Int32Value", int_payload);

  ASSERT_TRUE(test::wait_until(
    std::chrono::milliseconds(300),
    std::chrono::milliseconds(2),
    [&]() { return count.load() >= 1; }
  ));
  const int before = count.load();

  google::protobuf::StringValue str_value;
  str_value.set_value("not-int");
  std::string str_payload;
  ASSERT_TRUE(str_value.SerializeToString(&str_payload));
  mw->publish_serialized(topic, "google.protobuf.StringValue", str_payload);

  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(count.load(), before);
}

TEST_F(MiddlewareIntegrationTest, SubscribeAnyDropsUntilTypeRegisteredThenDelivers) {
  auto mw = Middleware::create();
  ASSERT_NE(mw, nullptr);

  const auto topic = test::unique_topic("/any");
  std::atomic<int> callback_count{0};
  std::atomic<int> last_value{-1};

  auto any_sub = mw->subscribe_any(
    topic,
    [&](const std::string&, const google::protobuf::Message& msg) {
      if (auto* int_msg = dynamic_cast<const google::protobuf::Int32Value*>(&msg)) {
        last_value.store(int_msg->value());
      }
      callback_count.fetch_add(1);
    }
  );
  ASSERT_TRUE(any_sub.valid());

  auto pub = detail::create_publisher_handle(topic, Qos::KeepLast, 16);
  ASSERT_TRUE(detail::publisher_valid(pub));

  google::protobuf::Int32Value raw_msg;
  raw_msg.set_value(7);
  std::string payload;
  ASSERT_TRUE(raw_msg.SerializeToString(&payload));
  ASSERT_TRUE(detail::publish_handle(pub, payload.data(), payload.size()));

  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(callback_count.load(), 0);

  mw->register_topic_type(topic, "google.protobuf.Int32Value");

  raw_msg.set_value(99);
  payload.clear();
  ASSERT_TRUE(raw_msg.SerializeToString(&payload));
  ASSERT_TRUE(detail::publish_handle(pub, payload.data(), payload.size()));

  ASSERT_TRUE(test::wait_until(
    std::chrono::milliseconds(300),
    std::chrono::milliseconds(2),
    [&]() { return callback_count.load() >= 1; }
  ));
  EXPECT_EQ(last_value.load(), 99);
}

TEST_F(MiddlewareIntegrationTest, NodePublisherSubscriptionBasicDelivery) {
  init();

  Node node("test_node");
  const auto topic = test::unique_topic("/node_int");

  auto pub = node.create_publisher<google::protobuf::Int32Value>(topic);
  ASSERT_TRUE(pub.valid());

  std::atomic<int> received{-1};
  auto sub = node.create_subscription<google::protobuf::Int32Value>(
    topic,
    [&](const google::protobuf::Int32Value& v) { received.store(v.value()); }
  );
  ASSERT_TRUE(sub.valid());

  google::protobuf::Int32Value v;
  v.set_value(41);
  pub.publish(v);
  v.set_value(42);
  pub.publish(v);

  ASSERT_TRUE(test::wait_until(
    std::chrono::milliseconds(300),
    std::chrono::milliseconds(2),
    [&]() { return received.load() == 42; }
  ));
}

TEST_F(MiddlewareIntegrationTest, RuntimeSpinExitsOnShutdown) {
  init();

  auto node = std::make_shared<Node>("spin_test");
  std::thread t([&]() { spin(node); });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  shutdown();
  t.join();

  SUCCEED();
}

TEST_F(MiddlewareIntegrationTest, RuntimeSpinCrossNodeCommunication) {
  init();

  auto node_pub = std::make_shared<Node>("pub_node");
  auto node_sub = std::make_shared<Node>("sub_node");
  const auto topic = test::unique_topic("/cross_node");

  auto pub = node_pub->create_publisher<google::protobuf::Int32Value>(topic);
  std::atomic<int> got{0};
  auto sub = node_sub->create_subscription<google::protobuf::Int32Value>(
    topic,
    [&](const google::protobuf::Int32Value& v) { got.store(v.value()); }
  );
  ASSERT_TRUE(pub.valid());
  ASSERT_TRUE(sub.valid());

  std::thread t([&]() { spin(std::vector<std::shared_ptr<Node>>{node_pub, node_sub}); });

  google::protobuf::Int32Value v;
  v.set_value(123);
  pub.publish(v);

  ASSERT_TRUE(test::wait_until(
    std::chrono::milliseconds(400),
    std::chrono::milliseconds(2),
    [&]() { return got.load() == 123; }
  ));

  shutdown();
  t.join();
}

} // namespace jr::mw
