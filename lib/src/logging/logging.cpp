// Implementation of the logging facade over DataTamer. All DataTamer / MCAP
// (C++17) code is confined to this translation unit.

#include "last_letter_lib/logging.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <map>
#include <memory>
#include <thread>

#include "data_tamer/data_tamer.hpp"
#include "data_tamer/sinks/mcap_sink.hpp"

namespace last_letter_lib {
namespace logging {

namespace {
bool g_enabled = false;
long long g_frame_index = 0; // Stamp for log_frame() with no time given.
long long g_frame_count = 0; // Frames written this recording; drives the empty-log warning.
unsigned g_epoch = 0;
// Keep the sink alive for the duration of the recording. Destroying it (in
// disable()) flushes and closes the MCAP file.
std::shared_ptr<DataTamer::MCAPSink> g_sink;

// The channels of the current recording, which log_frame() walks. We own them
// rather than using DataTamer::ChannelsRegistry because the registry offers no
// way to enumerate its channels and no way to erase one only
// getChannel/addDefaultSink/clear.
std::map<std::string, std::shared_ptr<DataTamer::LogChannel>> g_channels;

// Snapshot every channel at `stamp`.
void snapshot_all(std::chrono::nanoseconds stamp)
{
    for (auto &entry : g_channels)
    {
        entry.second->takeSnapshot(stamp);
    }
    ++g_frame_count;
}

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

    // Start from a clean slate: drop any channels from a previous run so each
    // recording is independent (important when many sims run in one process,
    // e.g. the test binary). This must precede replacing the sink, since the
    // channels hold a shared_ptr to it.
    g_channels.clear();

    // do_compression=false: more robust to crash/segfault mid-run, which
    // matters for long streaming sims.
    g_sink = std::make_shared<DataTamer::MCAPSink>(out_path, /*do_compression=*/false);
    // MCAPSink defaults to reopening the *same* file from scratch every 600 s, as a
    // guard against filling the disk. We would rather run out of disk than lose
    // data, so disable it: 0 means never reset.
    g_sink->setMaxTimeBeforeReset(std::chrono::seconds(0));

    g_enabled = true;
    g_frame_index = 0;
    g_frame_count = 0;
    ++g_epoch;
}

void disable()
{
    // Registering channels and never writing a frame is the one way to get a
    // valid but empty file out of this API, and nothing else complains about
    // it. Say so rather than leave it to be discovered at analysis time.
    if (g_enabled && g_frame_count == 0 && !g_channels.empty())
    {
        std::fprintf(stderr,
                     "last_letter logging: %zu channel(s) registered but log_frame() was "
                     "never called; the recording is empty.\n",
                     g_channels.size());
    }

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
    g_channels.clear();
    g_sink.reset(); // flush + close the MCAP file
    g_enabled = false;
}

bool is_enabled()
{
    return g_enabled;
}

void log_frame(double t_seconds)
{
    if (!g_enabled)
    {
        return;
    }
    snapshot_all(std::chrono::nanoseconds(std::llround(t_seconds * 1e9)));
}

void log_frame()
{
    if (!g_enabled)
    {
        return;
    }
    // Advance only when a frame is really written, so a stretch with logging
    // off does not leave a gap in the index.
    snapshot_all(std::chrono::nanoseconds(g_frame_index++));
}

unsigned epoch()
{
    return g_epoch;
}

LogChannel get_channel(const std::string &name)
{
    auto it = g_channels.find(name);
    if (it == g_channels.end())
    {
        auto channel = DataTamer::LogChannel::create(name);
        if (g_sink)
        {
            channel->addDataSink(g_sink);
        }
        it = g_channels.emplace(name, std::move(channel)).first;
    }
    // The shared_ptr<DataTamer::LogChannel> is stored type-erased in the
    // handle's shared_ptr<void>.
    return LogChannel(it->second);
}

void drop_channel(const std::string &name)
{
    g_channels.erase(name);
}

} // namespace logging
} // namespace last_letter_lib
