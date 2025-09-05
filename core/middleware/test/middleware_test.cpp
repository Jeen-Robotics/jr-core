#include <gtest/gtest.h>

#include "middleware/middleware.hpp"

#include <atomic>
#include <optional>
#include <string>
#include <thread>

namespace jr::mw {

TEST(Middleware, PublishSubscribe_DeliversLatest) {
  auto mw = Middleware::create();
  std::optional<int> received;
  auto sub = mw->subscribe<int>("/int", [&](const int& v) { received = v; });

  // Multiple publishes; keep-latest policy means last value is delivered
  // eventually
  mw->publish<int>("/int", 1);
  mw->publish<int>("/int", 2);
  mw->publish<int>("/int", 3);

  // Wait for dispatcher thread to deliver
  for (int i = 0; i < 50 && !received.has_value(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(received.has_value());
  EXPECT_EQ(*received, 3);
}

TEST(Middleware, Unsubscribe_StopsDelivery) {
  auto mw = Middleware::create();
  std::atomic<int> count{0};
  auto sub = mw->subscribe<int>("/i", [&](const int&) { ++count; });

  mw->publish<int>("/i", 42);
  // wait for first delivery
  for (int i = 0; i < 50 && count.load() == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_GE(count.load(), 1);

  sub.unsubscribe();
  const int before = count.load();
  mw->publish<int>("/i", 43);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(count.load(), before);
}

TEST(Middleware, MultipleSubscribers_AllReceive) {
  auto mw = Middleware::create();
  std::atomic<int> a{0};
  std::atomic<int> b{0};
  auto sa = mw->subscribe<int>("/t", [&](const int&) { ++a; });
  auto sb = mw->subscribe<int>("/t", [&](const int&) { ++b; });

  mw->publish<int>("/t", 7);
  for (int i = 0; i < 50 && (a.load() == 0 || b.load() == 0); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

  EXPECT_GE(a.load(), 1);
  EXPECT_GE(b.load(), 1);
}

TEST(Middleware, TypeIsolation_DifferentTypesSameTopic) {
  auto mw = Middleware::create();

  // First subscriber sets the topic type to int
  std::atomic<int> ints{0};
  auto si = mw->subscribe<int>("/topic", [&](const int&) { ++ints; });
  ASSERT_TRUE(si.valid());

  // Mismatched subscription should be invalid and not receive anything
  std::atomic<int> strings{0};
  auto ss = mw->subscribe<std::string>("/topic", [&](const std::string&) {
    ++strings;
  });
  EXPECT_FALSE(ss.valid());

  // Publishing int should deliver to int subscriber
  mw->publish<int>("/topic", 5);
  for (int i = 0; i < 50 && ints.load() == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_GE(ints.load(), 1);

  // Publishing string should be dropped silently
  mw->publish<std::string>("/topic", std::string("hello"));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(strings.load(), 0);
}

TEST(Middleware, Subscription_MoveSemantics) {
  auto mw = Middleware::create();
  std::atomic<int> cnt{0};
  Subscription s1;
  {
    Subscription temp = mw->subscribe<int>("/a", [&](const int&) { ++cnt; });
    EXPECT_TRUE(temp.valid());
    s1 = std::move(temp);
    EXPECT_FALSE(temp.valid());
    EXPECT_TRUE(s1.valid());
  }
  mw->publish<int>("/a", 1);
  for (int i = 0; i < 50 && cnt.load() == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_GE(cnt.load(), 1);

  Subscription s2 = std::move(s1);
  EXPECT_FALSE(s1.valid());
  EXPECT_TRUE(s2.valid());

  s2.unsubscribe();
  int before = cnt.load();
  mw->publish<int>("/a", 2);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(cnt.load(), before);
}

TEST(Middleware, KeepLatest_DropsOldQueueEntries) {
  auto mw = Middleware::create();
  std::atomic<int> last{-1};
  auto s = mw->subscribe<int>("/k", [&](const int& v) { last.store(v); });

  // Publish many without waiting; queue is size 1 (drop oldest)
  for (int i = 0; i < 20; ++i) {
    mw->publish<int>("/k", i);
  }

  // Eventually the latest should be delivered
  for (int i = 0; i < 200 && last.load() != 19; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(last.load(), 19);
}

TEST(Node, PublisherSubscription_BasicDelivery) {
  init();
  auto node = Node("test");
  auto pub = node.create_publisher<int>("/n/int");
  std::optional<int> received;
  auto sub = node.create_subscription<int>("/n/int", [&](const int& v) {
    received = v;
  });

  pub.publish(41);
  pub.publish(42);

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
  auto pub = node.create_publisher<std::string>("/n/str");
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

  auto pub = node_pub->create_publisher<int>("/x");
  std::atomic<int> got{0};
  auto sub = node_sub->create_subscription<int>("/x", [&](const int& v) { got = v; });

  std::thread t([&]() { spin(std::vector<std::shared_ptr<Node>>{node_pub, node_sub}); });
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  pub.publish(123);

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
      , pub_(create_publisher<int>("/ros_like")) {
  }

  void send(int value) { pub_.publish(value); }

private:
  Publisher<int> pub_;
};

class SubscriberNode : public Node {
public:
  explicit SubscriberNode(std::string name)
      : Node(std::move(name))
      , sub_(create_subscription<int>("/ros_like", [&](const int& v) {
          last_.store(v);
        })) {
  }

  int last() const { return last_.load(); }

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
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  pub->send(777);

  for (int i = 0; i < 100 && sub->last() != 777; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(sub->last(), 777);
  shutdown();
  t.join();
}

} // namespace jr::mw
