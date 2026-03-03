#include <gtest/gtest.h>

#include <middleware/backend.hpp>
#include <middleware/middleware.hpp>

namespace jr::mw {

class BackendUnitTest : public ::testing::Test {
protected:
  void TearDown() override {
    shutdown();
  }
};

TEST_F(BackendUnitTest, BackendNameKnownValues) {
  EXPECT_STREQ(backend_name(Backend::Rust), "rust");
  EXPECT_STREQ(backend_name(Backend::Cpp), "cpp");
}

TEST_F(BackendUnitTest, SetBackendToCurrentIsAllowedAfterInit) {
  ASSERT_TRUE(init());
  const auto current = get_backend();
  EXPECT_TRUE(set_backend(current));
}

TEST_F(BackendUnitTest, SetBackendToDifferentIsRejectedAfterInit) {
  ASSERT_TRUE(init());
  const auto current = get_backend();
  const auto other = (current == Backend::Rust) ? Backend::Cpp : Backend::Rust;

  EXPECT_FALSE(set_backend(other));
  EXPECT_EQ(get_backend(), current);
}

TEST_F(BackendUnitTest, GlobalGetShutdownLifecycle) {
  auto first = get();
  ASSERT_NE(first, nullptr);

  shutdown();

  auto second = get();
  ASSERT_NE(second, nullptr);
  EXPECT_NE(first.get(), second.get());
}

} // namespace jr::mw
