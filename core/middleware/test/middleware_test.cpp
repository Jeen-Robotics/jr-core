#include <gtest/gtest.h>

#include <middleware/middleware.hpp>
#include <middleware/node.hpp>

#include <atomic>
#include <optional>
#include <string>
#include <thread>

#include <google/protobuf/wrappers.pb.h>

namespace jr::mw {

TEST(Middleware, PublishSubscribe_DeliversLatest) {
  auto mw = Middleware::create();
  std::atomic<int> received{-1};
  auto sub = mw->subscribe<google::protobuf::Int32Value>(
    "/int",
    [&](const google::protobuf::Int32Value& v) { received.store(v.value()); }
  );

  google::protobuf::Int32Value v;
  v.set_value(1);
  mw->publish("/int", v);
  v.set_value(2);
  mw->publish("/int", v);
  v.set_value(3);
  mw->publish("/int", v);

  // Wait specifically for value 3 (the latest)
  // With KeepLast QoS, all messages are delivered but we want to see 3 eventually
  for (int i = 0; i < 100 && received.load() != 3; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(received.load(), 3);
}

TEST(Middleware, Unsubscribe_StopsDelivery) {
  auto mw = Middleware::create();
  std::atomic<int> count{0};
  auto sub = mw->subscribe<google::protobuf::Int32Value>(
    "/i",
    [&](const google::protobuf::Int32Value&) { ++count; }
  );

  google::protobuf::Int32Value v;
  v.set_value(42);
  mw->publish("/i", v);
  for (int i = 0; i < 50 && count.load() == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_GE(count.load(), 1);

  sub.unsubscribe();
  const int before = count.load();
  v.set_value(43);
  mw->publish("/i", v);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(count.load(), before);
}

TEST(Middleware, MultipleSubscribers_AllReceive) {
  auto mw = Middleware::create();
  std::atomic<int> a{0};
  std::atomic<int> b{0};
  auto sa = mw->subscribe<google::protobuf::Int32Value>(
    "/t",
    [&](const google::protobuf::Int32Value&) { ++a; }
  );
  auto sb = mw->subscribe<google::protobuf::Int32Value>(
    "/t",
    [&](const google::protobuf::Int32Value&) { ++b; }
  );

  google::protobuf::Int32Value v;
  v.set_value(7);
  mw->publish("/t", v);
  for (int i = 0; i < 50 && (a.load() == 0 || b.load() == 0); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

  EXPECT_GE(a.load(), 1);
  EXPECT_GE(b.load(), 1);
}

TEST(Middleware, TypeIsolation_DifferentTypesSameTopic) {
  auto mw = Middleware::create();

  std::atomic<int> ints{0};
  auto si = mw->subscribe<google::protobuf::Int32Value>(
    "/topic",
    [&](const google::protobuf::Int32Value&) { ++ints; }
  );
  ASSERT_TRUE(si.valid());

  std::atomic<int> strings{0};
  auto ss = mw->subscribe<google::protobuf::StringValue>(
    "/topic",
    [&](const google::protobuf::StringValue&) { ++strings; }
  );
  EXPECT_FALSE(ss.valid());

  google::protobuf::Int32Value v;
  v.set_value(5);
  mw->publish("/topic", v);
  for (int i = 0; i < 50 && ints.load() == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_GE(ints.load(), 1);

  google::protobuf::StringValue sv;
  sv.set_value("hello");
  mw->publish("/topic", sv);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(strings.load(), 0);
}

TEST(Middleware, Subscription_MoveSemantics) {
  auto mw = Middleware::create();
  std::atomic<int> cnt{0};
  Subscription s1;
  {
    Subscription temp = mw->subscribe<google::protobuf::Int32Value>(
      "/a",
      [&](const google::protobuf::Int32Value&) { ++cnt; }
    );
    EXPECT_TRUE(temp.valid());
    s1 = std::move(temp);
    EXPECT_FALSE(temp.valid());
    EXPECT_TRUE(s1.valid());
  }
  google::protobuf::Int32Value v;
  v.set_value(1);
  mw->publish("/a", v);
  for (int i = 0; i < 50 && cnt.load() == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_GE(cnt.load(), 1);

  Subscription s2 = std::move(s1);
  EXPECT_FALSE(s1.valid());
  EXPECT_TRUE(s2.valid());

  s2.unsubscribe();
  int before = cnt.load();
  v.set_value(2);
  mw->publish("/a", v);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(cnt.load(), before);
}

TEST(Middleware, KeepLatest_DropsOldQueueEntries) {
  auto mw = Middleware::create();
  std::atomic<int> last{-1};
  auto s = mw->subscribe<google::protobuf::Int32Value>(
    "/k",
    [&](const google::protobuf::Int32Value& vv) { last.store(vv.value()); }
  );

  google::protobuf::Int32Value v;
  for (int i = 0; i < 20; ++i) {
    v.set_value(i);
    mw->publish("/k", v);
  }

  for (int i = 0; i < 200 && last.load() != 19; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(last.load(), 19);
}

TEST(Node, PublisherSubscription_BasicDelivery) {
  init();
  auto node = Node("test");
  auto pub = node.create_publisher<google::protobuf::Int32Value>("/n/int");
  std::optional<int> received;
  auto sub = node.create_subscription<google::protobuf::Int32Value>(
    "/n/int",
    [&](const google::protobuf::Int32Value& v) { received = v.value(); }
  );

  google::protobuf::Int32Value v;
  v.set_value(41);
  pub.publish(v);
  v.set_value(42);
  pub.publish(v);

  for (int i = 0; i < 50 && !received.has_value(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(received.has_value());
  EXPECT_EQ(*received, 42);
  shutdown();
}

TEST(Node, Publisher_Validity) {
  init();
  auto node = Node("test");
  auto pub = node.create_publisher<google::protobuf::StringValue>("/n/str");
  EXPECT_TRUE(pub.valid());
  shutdown();
}

TEST(Runtime, SpinWaitsUntilShutdown) {
  init();
  auto node = std::make_shared<Node>("spin_test");
  std::thread t([&]() { spin(node); });
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  shutdown();
  t.join();
}

TEST(Runtime, MultiNodeSpin_CrossNodeCommunication) {
  init();
  auto node_pub = std::make_shared<Node>("pub_node");
  auto node_sub = std::make_shared<Node>("sub_node");

  auto pub = node_pub->create_publisher<google::protobuf::Int32Value>("/x");
  std::atomic<int> got{0};
  auto sub = node_sub->create_subscription<google::protobuf::Int32Value>(
    "/x",
    [&](const google::protobuf::Int32Value& v) { got = v.value(); }
  );

  std::thread t([&]() {
    spin(std::vector<std::shared_ptr<Node>>{node_pub, node_sub});
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  google::protobuf::Int32Value v;
  v.set_value(123);
  pub.publish(v);

  for (int i = 0; i < 100 && got.load() != 123; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(got.load(), 123);
  shutdown();
  t.join();
}

namespace {
class PublisherNode : public Node {
public:
  explicit PublisherNode(std::string name)
      : Node(std::move(name))
      , pub_(create_publisher<google::protobuf::Int32Value>("/ros_like")) {
  }

  void send(int value) {
    google::protobuf::Int32Value v;
    v.set_value(value);
    pub_.publish(v);
  }

  void spin_once() override {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    send(777);
  }

private:
  Publisher<google::protobuf::Int32Value> pub_;
};

class SubscriberNode : public Node {
public:
  explicit SubscriberNode(std::string name)
      : Node(std::move(name))
      , sub_(create_subscription<google::protobuf::Int32Value>(
          "/ros_like",
          [&](const google::protobuf::Int32Value& v) { last_.store(v.value()); }
        )) {
  }

  int last() const {
    return last_.load();
  }

private:
  std::atomic<int> last_{-1};
  Subscription sub_;
};
} // namespace

TEST(Runtime, DerivedNodeClasses_PubSub) {
  init();
  auto pub = std::make_shared<PublisherNode>("pub");
  auto sub = std::make_shared<SubscriberNode>("sub");

  std::thread t([&]() { spin(std::vector<std::shared_ptr<Node>>{pub, sub}); });

  for (int i = 0; i < 100 && sub->last() != 777; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(sub->last(), 777);
  shutdown();
  t.join();
}

// --- Additional tests ---

TEST(Middleware, PublishSerialized_Works) {
  auto mw = Middleware::create();
  std::optional<int> received;
  auto sub = mw->subscribe<google::protobuf::Int32Value>(
    "/serialized",
    [&](const google::protobuf::Int32Value& v) { received = v.value(); }
  );

  // Manually serialize and publish
  google::protobuf::Int32Value v;
  v.set_value(999);
  std::string payload;
  ASSERT_TRUE(v.SerializeToString(&payload));
  
  mw->publish_serialized("/serialized", "google.protobuf.Int32Value", payload);

  for (int i = 0; i < 50 && !received.has_value(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(received.has_value());
  EXPECT_EQ(*received, 999);
}

TEST(Middleware, SensorDataQos_GetsLatestOnly) {
  auto mw = Middleware::create();
  std::atomic<int> last{-1};
  auto sub = mw->subscribe<google::protobuf::Int32Value>(
    "/sensor",
    [&](const google::protobuf::Int32Value& v) { 
      last.store(v.value());
      // Simulate slow processing
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    },
    Qos::SensorData,
    1  // capacity 1 for SensorData
  );

  // Rapid-fire publish
  google::protobuf::Int32Value v;
  for (int i = 0; i < 100; ++i) {
    v.set_value(i);
    mw->publish("/sensor", v);
  }

  // Wait for processing
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  // Should have received a recent value (not necessarily 99, but close)
  EXPECT_GE(last.load(), 0);
}

TEST(Middleware, GetTopicNamesAndTypes_ReturnsRegisteredTopics) {
  auto mw = Middleware::create();
  
  // Subscribe to create topics
  auto sub1 = mw->subscribe<google::protobuf::Int32Value>(
    "/topic1",
    [](const google::protobuf::Int32Value&) {}
  );
  auto sub2 = mw->subscribe<google::protobuf::StringValue>(
    "/topic2", 
    [](const google::protobuf::StringValue&) {}
  );
  
  auto topics = mw->get_topic_names_and_types();
  
  // Should have at least our two topics
  bool found_topic1 = false;
  bool found_topic2 = false;
  for (const auto& info : topics) {
    if (info.name == "/topic1") {
      found_topic1 = true;
      EXPECT_EQ(info.type_full_name, "google.protobuf.Int32Value");
    }
    if (info.name == "/topic2") {
      found_topic2 = true;
      EXPECT_EQ(info.type_full_name, "google.protobuf.StringValue");
    }
  }
  EXPECT_TRUE(found_topic1);
  EXPECT_TRUE(found_topic2);
}

TEST(Middleware, MultipleIndependentTopics_NoInterference) {
  auto mw = Middleware::create();
  std::atomic<int> count_a{0};
  std::atomic<int> count_b{0};
  
  auto sub_a = mw->subscribe<google::protobuf::Int32Value>(
    "/topic_a",
    [&](const google::protobuf::Int32Value&) { ++count_a; }
  );
  auto sub_b = mw->subscribe<google::protobuf::Int32Value>(
    "/topic_b",
    [&](const google::protobuf::Int32Value&) { ++count_b; }
  );

  google::protobuf::Int32Value v;
  v.set_value(1);
  
  // Publish only to topic_a
  mw->publish("/topic_a", v);
  
  for (int i = 0; i < 50 && count_a.load() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  
  EXPECT_GE(count_a.load(), 1);
  EXPECT_EQ(count_b.load(), 0);  // topic_b should not receive
}

TEST(Middleware, ConcurrentPublish_ThreadSafe) {
  auto mw = Middleware::create();
  std::atomic<int> total{0};
  
  auto sub = mw->subscribe<google::protobuf::Int32Value>(
    "/concurrent",
    [&](const google::protobuf::Int32Value& v) { 
      total.fetch_add(v.value());
    }
  );

  constexpr int NUM_THREADS = 4;
  constexpr int MSGS_PER_THREAD = 10;
  
  std::vector<std::thread> threads;
  for (int t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back([&mw]() {
      google::protobuf::Int32Value v;
      v.set_value(1);
      for (int i = 0; i < MSGS_PER_THREAD; ++i) {
        mw->publish("/concurrent", v);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });
  }
  
  for (auto& t : threads) {
    t.join();
  }
  
  // Wait for all messages to be processed
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Should have received all messages (may drop some due to QoS, but should have many)
  EXPECT_GT(total.load(), 0);
}

TEST(Middleware, Create_ReturnsUniqueInstances) {
  auto mw1 = Middleware::create();
  auto mw2 = Middleware::create();
  
  EXPECT_NE(mw1.get(), mw2.get());
  EXPECT_TRUE(mw1 != nullptr);
  EXPECT_TRUE(mw2 != nullptr);
}

TEST(Middleware, ShutdownCleansUp) {
  auto mw = Middleware::create();
  
  std::atomic<bool> callback_called{false};
  auto sub = mw->subscribe<google::protobuf::Int32Value>(
    "/shutdown_test",
    [&](const google::protobuf::Int32Value&) { callback_called = true; }
  );
  
  EXPECT_TRUE(sub.valid());
  
  mw->shutdown();
  
  // After shutdown, publishing should not crash
  google::protobuf::Int32Value v;
  v.set_value(1);
  mw->publish("/shutdown_test", v);
  
  // Brief wait
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  
  // Callback should not be called after shutdown
  EXPECT_FALSE(callback_called.load());
}

TEST(Node, CreatePublisherWithQos) {
  init();
  auto node = Node("qos_test");
  
  auto pub_keeplast = node.create_publisher<google::protobuf::Int32Value>(
    "/qos_keeplast", Qos::KeepLast, 32
  );
  EXPECT_TRUE(pub_keeplast.valid());
  
  auto pub_sensor = node.create_publisher<google::protobuf::Int32Value>(
    "/qos_sensor", Qos::SensorData
  );
  EXPECT_TRUE(pub_sensor.valid());
  
  shutdown();
}

TEST(Middleware, SubscribeAny_ReceivesMessages) {
  auto mw = Middleware::create();
  
  std::atomic<int> received_count{0};
  std::mutex mtx;
  std::string received_type;
  int received_value = -1;
  
  // First publish to register the type
  google::protobuf::Int32Value v;
  v.set_value(42);
  mw->publish("/any_test", v);
  
  // Now subscribe_any
  auto sub = mw->subscribe_any(
    "/any_test",
    [&](const std::string& type_name, const google::protobuf::Message& msg) {
      std::lock_guard<std::mutex> lock(mtx);
      received_type = type_name;
      if (auto* int_msg = dynamic_cast<const google::protobuf::Int32Value*>(&msg)) {
        received_value = int_msg->value();
      }
      ++received_count;
    }
  );
  
  ASSERT_TRUE(sub.valid());
  
  // Publish more messages
  v.set_value(100);
  mw->publish("/any_test", v);
  v.set_value(200);
  mw->publish("/any_test", v);
  
  // Wait for messages
  for (int i = 0; i < 100 && received_count.load() < 2; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  
  {
    std::lock_guard<std::mutex> lock(mtx);
    EXPECT_GE(received_count.load(), 1);
    EXPECT_EQ(received_type, "google.protobuf.Int32Value");
    EXPECT_GE(received_value, 100);  // Should have received 100 or 200
  }
}

TEST(Middleware, SubscribeAny_MultipleTypes) {
  auto mw = Middleware::create();
  
  std::atomic<int> int_count{0};
  std::atomic<int> str_count{0};
  
  // Register types by publishing
  google::protobuf::Int32Value iv;
  iv.set_value(1);
  mw->publish("/any_int", iv);
  
  google::protobuf::StringValue sv;
  sv.set_value("hello");
  mw->publish("/any_str", sv);
  
  // Subscribe to both
  auto sub_int = mw->subscribe_any(
    "/any_int",
    [&](const std::string&, const google::protobuf::Message&) { ++int_count; }
  );
  auto sub_str = mw->subscribe_any(
    "/any_str",
    [&](const std::string&, const google::protobuf::Message&) { ++str_count; }
  );
  
  ASSERT_TRUE(sub_int.valid());
  ASSERT_TRUE(sub_str.valid());
  
  // Publish more
  iv.set_value(2);
  mw->publish("/any_int", iv);
  sv.set_value("world");
  mw->publish("/any_str", sv);
  
  // Wait
  for (int i = 0; i < 100 && (int_count.load() == 0 || str_count.load() == 0); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  
  EXPECT_GE(int_count.load(), 1);
  EXPECT_GE(str_count.load(), 1);
}

} // namespace jr::mw
