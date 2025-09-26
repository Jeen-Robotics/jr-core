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
  std::optional<int> received;
  auto sub = mw->subscribe<google::protobuf::Int32Value>(
    "/int",
    [&](const google::protobuf::Int32Value& v) { received = v.value(); }
  );

  google::protobuf::Int32Value v;
  v.set_value(1);
  mw->publish("/int", v);
  v.set_value(2);
  mw->publish("/int", v);
  v.set_value(3);
  mw->publish("/int", v);

  for (int i = 0; i < 50 && !received.has_value(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(received.has_value());
  EXPECT_EQ(*received, 3);
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

} // namespace jr::mw
