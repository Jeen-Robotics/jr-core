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

} // namespace jr::mw
