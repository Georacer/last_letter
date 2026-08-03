// Implementation of the logging facade over DataTamer. All DataTamer / MCAP
// (C++17) code is confined to this translation unit.

#include "last_letter_lib/logging.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <memory>
#include <thread>

#include "data_tamer/data_tamer.hpp"
#include "data_tamer/sinks/mcap_sink.hpp"

namespace last_letter_lib {
namespace logging {

namespace {
bool g_enabled = false;
bool g_time_set = false;
long long g_time_ns = 0;
long long g_auto_index = 0;
unsigned g_epoch = 0;
// Keep the sink alive for the duration of the recording. Destroying it (in
// disable()) flushes and closes the MCAP file.
std::shared_ptr<DataTamer::MCAPSink> g_sink;

// Filename derived from the current local date/time, e.g.
// "2026-07-24_01-30-00.mcap". Used when enable() is called with no path.
std::string default_log_filename()
{
    std::time_t now = std::time(nullptr);
    std::tm local_tm{};
    localtime_r(&now, &local_tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S.mcap", &local_tm);
    return std::string(buf);
}
} // namespace

void enable(const std::string &path)
{
    // With no path, name the file after the current local date/time so repeated
    // runs don't clobber each other (written to the current working directory).
    const std::string out_path = path.empty() ? default_log_filename() : path;

    // Start from a clean slate: drop any channels/sinks from a previous run so
    // each recording is independent (important when many sims run in one
    // process, e.g. the test binary).
    DataTamer::ChannelsRegistry::Global().clear();

    // do_compression=false: more robust to crash/segfault mid-run, which
    // matters for long streaming sims.
    g_sink = std::make_shared<DataTamer::MCAPSink>(out_path, /*do_compression=*/false);
    // MCAPSink defaults to reopening the *same* file from scratch every 600 s, as a
    // guard against filling the disk. We would rather run out of disk than lose
    // data, so disable it: 0 means never reset.
    g_sink->setMaxTimeBeforeReset(std::chrono::seconds(0));
    DataTamer::ChannelsRegistry::Global().addDefaultSink(g_sink);

    g_enabled = true;
    g_time_set = false;
    g_auto_index = 0;
    ++g_epoch;
}

void disable()
{
    // DataTamer's sink writes on a background thread that drains its queue in
    // ~250us passes, and its shutdown (stopThread) does NOT drain the queue
    // first. So we must let the worker flush pending snapshots before tearing
    // the sink down, otherwise the most recent samples are lost. One worker
    // pass drains the whole queue; this delay covers many passes with margin
    // for scheduler jitter, and is a one-time cost paid only at disable().
    if (g_sink)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    DataTamer::ChannelsRegistry::Global().clear();
    g_sink.reset(); // flush + close the MCAP file
    g_enabled = false;
}

bool is_enabled()
{
    return g_enabled;
}

void set_time(double t_seconds)
{
    g_time_ns = static_cast<long long>(std::llround(t_seconds * 1e9));
    g_time_set = true;
}

long long now_ns()
{
    return g_time_set ? g_time_ns : (g_auto_index++);
}

unsigned epoch()
{
    return g_epoch;
}

LogChannel get_channel(const std::string &name)
{
    // getChannel() returns shared_ptr<DataTamer::LogChannel>, stored type-erased
    // in the handle's shared_ptr<void>.
    return LogChannel(DataTamer::ChannelsRegistry::Global().getChannel(name));
}

void LogChannel::take_snapshot()
{
    if (!impl_)
    {
        return;
    }
    std::static_pointer_cast<DataTamer::LogChannel>(impl_)
        ->takeSnapshot(std::chrono::nanoseconds(now_ns()));
}

} // namespace logging
} // namespace last_letter_lib
