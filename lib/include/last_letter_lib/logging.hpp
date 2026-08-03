#pragma once

// Data-logging facade for last_letter_lib.
//
// This header is intentionally free of any DataTamer / MCAP includes and uses
// only C++11-clean types in its signatures. All backend (DataTamer, C++17)
// code is confined to src/logging/logging.cpp. This keeps the header safe to
// include from a C++11 translation unit (e.g. an ArduPilot-facing facade).
//
// Logging is a process-global concern, off by default. Enabling attaches an
// MCAP sink to DataTamer's global channel registry; every Component / UavModel
// then self-logs into that sink from inside its calc_model()/step(). When
// logging is disabled, the self-log is a cheap no-op.

#include <memory>
#include <string>

namespace last_letter_lib
{
namespace logging
{

// Start a new recording, writing to `path` (should end in ".mcap"). If `path`
// is empty (the default), a filename is generated from the current local
// date/time as "YYYY-MM-DD_HH-MM-SS.mcap" in the current working directory, so
// repeated runs do not overwrite each other.
// Clears any previous recording and bumps the epoch (see epoch()).
// Call this BEFORE the first calc_model()/step() so channels are created with
// the sink already attached.
void enable(const std::string &path = "");

// Stop the current recording, flushing and closing the file.
void disable();

// True while a recording is active.
bool is_enabled();

// Set the timestamp (in seconds) applied to every subsequent snapshot until
// changed. The full model calls this once per step(); standalone drivers may
// call it too. If never called, snapshots use an auto-incrementing sample
// index instead (sufficient for a single-component recording).
void set_time(double t_seconds);

// Current snapshot timestamp in nanoseconds. Used internally by the loggers.
// Returns the value set by set_time(), or an auto-incrementing counter.
long long now_ns();

// Monotonically increasing recording id, bumped by each enable(). Loggers use
// it to know when to (re)register their channels for a fresh recording.
unsigned epoch();

// Opaque handle to one named log channel. It hides the logging backend from
// callers: obtain it with get_channel(), register the entity's members on it,
// then take_snapshot(). Copying a handle refers to the same underlying channel.
class LogChannel
{
  public:
    LogChannel() = default;

    // Register a loggable member by address; the pointee must outlive the
    // recording. Defined in log_types.hpp (the backend-aware header), so this
    // header stays free of the backend and safe to include from C++11 code.
    template <typename T>
    void register_value(const std::string &name, const T *value);

    // Capture all registered values into one snapshot, timestamped with now_ns().
    void take_snapshot();

    // False for a default-constructed / empty handle.
    explicit operator bool() const { return static_cast<bool>(impl_); }

  private:
    friend LogChannel get_channel(const std::string &);
    explicit LogChannel(std::shared_ptr<void> impl) : impl_(std::move(impl)) {}

    // Type-erased std::shared_ptr<DataTamer::LogChannel>; the backend type is
    // never named in this header. Recovered via static_pointer_cast in the
    // backend TUs (logging.cpp, log_types.hpp).
    std::shared_ptr<void> impl_;
};

// Get (creating on first use) the named channel from the global registry.
LogChannel get_channel(const std::string &name);

} // namespace logging
} // namespace last_letter_lib
