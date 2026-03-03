#pragma once

#include <cstdint>

namespace jr::mw {

/// Middleware transport backend.
enum class Backend : std::uint8_t {
  Rust = 0,
  Cpp = 1,
};

/// Select backend implementation.
/// Must be called before middleware is initialized.
/// @return false if backend is already locked to a different value.
bool set_backend(Backend backend);

/// Get currently selected backend.
Backend get_backend();

/// Convert backend enum to string ("rust" or "cpp").
const char* backend_name(Backend backend);

/// Initialize selected backend (idempotent).
bool init();

} // namespace jr::mw
