#pragma once

/// @file publisher.hpp
/// @brief Typed publisher for protobuf messages
///
/// Re-exports the Publisher from middleware_rs for backward compatibility.

#include <middleware_rs/publisher.hpp>

// The Publisher<T> class is now provided by middleware_rs.
// This header exists for backward compatibility with existing code
// that includes <middleware/publisher.hpp>.

namespace jr::mw {

// Publisher<T> is already defined in jr::mw namespace by middleware_rs/publisher.hpp

} // namespace jr::mw
