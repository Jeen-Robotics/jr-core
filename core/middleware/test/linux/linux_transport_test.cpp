#include <gtest/gtest.h>

#include <middleware/backend.hpp>
#include <middleware/detail/transport.hpp>
#include <middleware/middleware.hpp>

#include "../test_common.hpp"

namespace jr::mw {

TEST(LinuxTransport, RustBackendProvidesPollableFd) {
  ASSERT_TRUE(set_backend(Backend::Rust));
  ASSERT_TRUE(init());

  const auto topic = test::unique_topic("/linux_fd");
  auto sub = detail::create_subscriber_handle(topic, Qos::KeepLast, 16);
  ASSERT_TRUE(detail::subscriber_valid(sub));

  const int fd = detail::subscriber_fd(sub);
  EXPECT_GE(fd, 0);

  shutdown();
}

} // namespace jr::mw
