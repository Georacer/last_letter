#pragma once

// Data-logging facade for last_letter_lib.
//
// This header is intentionally free of any DataTamer / MCAP includes and uses
// only C++11-clean types in its signatures. All backend (DataTamer, C++17)
// code is confined to src/logging/logging.cpp. This keeps the header safe to
// include from a C++11 translation unit (e.g. an ArduPilot-facing facade).
//
// Logging is a process-global concern, off by default. Enabling opens an MCAP
// sink; every Component / UavModel then registers its members into a named
// channel from inside its calc_model()/step(). Registering does not write
// anything: one call to log_frame() snapshots *every* registered channel at
// *one* timestamp, which is what keeps the channels of a log aligned on a
// common time axis. UavModel::step() calls log_frame() itself; a driver
// stepping bare Components must call it once per frame. When logging is
// disabled, all of this is a cheap no-op.

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

// Write one snapshot of EVERY registered channel, all stamped `t_seconds`.
// No-op when logging is off.
void log_frame(double t_seconds);

// As above, but stamped with a frame index instead of a simulation time. The
// index starts at 0 for each recording and advances only when a frame is
// actually written. Use it when there is no meaningful clock to report.
void log_frame();

// Monotonically increasing recording id, bumped by each enable(). Loggers use
// it to know when to (re)register their channels for a fresh recording.
unsigned epoch();

// Opaque handle to one named log channel. It hides the logging backend from
// callers: obtain it with get_channel() and register the entity's members on
// it. The values are then read out of the entity by every log_frame(), so the
// handle itself takes no snapshots. Copying a handle refers to the same
// underlying channel.
class LogChannel
{
  public:
    LogChannel() = default;

    // Register a loggable member by address; the pointee must outlive the
    // channel, since every log_frame() reads through this pointer. An entity
    // that dies first must drop_channel() from its destructor. Defined in
    // log_types.hpp (the backend-aware header), so this header stays free of
    // the backend and safe to include from C++11 code.
    template <typename T>
    void register_value(const std::string &name, const T *value);

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

// Get (creating on first use) the named channel. Channel names must be unique
// among simultaneously live entities: two of them registering the same field
// name on one channel is an error the backend throws on.
LogChannel get_channel(const std::string &name);

// Destroy the named channel and take it out of the set log_frame() writes.
// Any entity that registered members of itself MUST call this from its
// destructor, or the next log_frame() reads freed memory through the pointers
// it registered. A no-op for an unknown name.
void drop_channel(const std::string &name);

} // namespace logging
} // namespace last_letter_lib
