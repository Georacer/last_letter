/*
  last_letter_ardupilot: drives a last_letter UavModel from ArduPilot SITL over
  the JSON backend protocol.

  ArduPilot's SITL::last_letter backend launches this program and then blocks on
  its UDP socket every frame, so the pair runs in lock-step: exactly one
  simulation step is taken per servo packet received.
*/

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

#include "last_letter_lib/ardupilot/json_connector.hpp"
#include "last_letter_lib/uav_model.hpp"

using last_letter_lib::UavModel;
using last_letter_lib::ardupilot::FrameKind;
using last_letter_lib::ardupilot::is_driven;
using last_letter_lib::ardupilot::JsonConnector;
using last_letter_lib::ardupilot::ServoFrame;
using last_letter_lib::ardupilot::StateFrame;
using last_letter_lib::programming_utils::loadModelConfig;
using last_letter_lib::uav_utils::Airdata;
using last_letter_lib::uav_utils::InputPwm_t;

namespace {

std::atomic<bool> g_stop{false};

void handle_signal(int /*signum*/)
{
    g_stop = true;
}

void usage(const char *argv0)
{
    printf("usage: %s [options]\n"
           "  --model <name>       aircraft model to simulate (default: ardupilot_plane)\n"
           "  --port <n>           UDP port to listen on (default: 9002)\n"
           "  --address <ip>       address to bind to (default: 0.0.0.0)\n"
           "  --models-dir <path>  models root (default: $HOME/last_letter_models)\n"
           "  --log [file.mcap]    record the run to an MCAP log; with no filename,\n"
           "                       a timestamped one in the working directory\n"
           "  --help\n",
           argv0);
}

struct Options
{
    std::string model{"ardupilot_plane"};
    std::string address{"0.0.0.0"};
    std::string models_dir;
    // log_path stays empty for a bare --log, which is how UavModel::enable_logging()
    // asks for a timestamped filename. want_log is what tells the two apart.
    std::string log_path;
    bool want_log{false};
    uint16_t port{9002};
};

// Returns false when the arguments are malformed.
bool parse_args(int argc, char **argv, Options &opts, bool &want_help)
{
    for (int i = 1; i < argc; i++)
    {
        const std::string arg{argv[i]};
        if (arg == "--help" || arg == "-h")
        {
            want_help = true;
            return true;
        }
        // The next element is this option's value unless there is none left or it
        // is itself an option. No option here takes a negative value, so a leading
        // '-' is unambiguous.
        const bool has_value = (i + 1 < argc) && (argv[i + 1][0] != '-');
        const std::string value{has_value ? argv[++i] : ""};

        // Section for flags with optional arguments.
        if (arg == "--log")
        {
            // The one option whose value is optional.
            opts.want_log = true;
            opts.log_path = value;
            continue;
        }
        // Section for flags with mandatory arguments.
        if (!has_value)
        {
            fprintf(stderr, "missing value for %s\n", arg.c_str());
            return false;
        }
        if (arg == "--model")
        {
            opts.model = value;
        }
        else if (arg == "--port")
        {
            opts.port = (uint16_t)atoi(value.c_str());
        }
        else if (arg == "--address")
        {
            opts.address = value;
        }
        else if (arg == "--models-dir")
        {
            opts.models_dir = value;
        }
        else
        {
            fprintf(stderr, "unknown option %s\n", arg.c_str());
            return false;
        }
    }
    return true;
}

// Fill the JSON payload from the model, right after a step().
//
// Frame conventions (see ardupilot/libraries/SITL/examples/JSON/readme.md):
//  - gyro and accel_body are body frame (FRD), everything else is NED;
//  - accel_body is the specific force an accelerometer would measure, so the
//    gravity contribution has to be removed.
void fill_state(const UavModel &uav, double timestamp, StateFrame &out)
{
    const auto &state = uav.state;

    out.timestamp = timestamp;

    out.gyro[0] = state.velocity.angular.x();
    out.gyro[1] = state.velocity.angular.y();
    out.gyro[2] = state.velocity.angular.z();

    const auto &wrenches = uav.dynamics.wrench_sum;
    const Eigen::Vector3d specific_force =
        (wrenches.sum().force - wrenches.wrenchGrav.force) / uav.dynamics.inertial.mass;
    out.accel_body[0] = specific_force.x();
    out.accel_body[1] = specific_force.y();
    out.accel_body[2] = specific_force.z();

    out.position[0] = state.pose.position.x();
    out.position[1] = state.pose.position.y();
    out.position[2] = state.pose.position.z();

    out.quaternion[0] = state.pose.orientation.w();
    out.quaternion[1] = state.pose.orientation.x();
    out.quaternion[2] = state.pose.orientation.y();
    out.quaternion[3] = state.pose.orientation.z();

    // velocity.linear is body-frame; the orientation quaternion rotates it to NED.
    const Eigen::Vector3d velocity_ned = state.pose.orientation * state.velocity.linear;
    out.velocity[0] = velocity_ned.x();
    out.velocity[1] = velocity_ned.y();
    out.velocity[2] = velocity_ned.z();

    const Eigen::Vector3d wind_ned = uav.environmentModel.environment.wind;
    out.velocity_wind[0] = wind_ned.x();
    out.velocity_wind[1] = wind_ned.y();
    out.velocity_wind[2] = wind_ned.z();

    // init_from_state_wind takes the wind in NED and rotates it to body itself.
    Airdata airdata;
    airdata.init_from_state_wind(state, wind_ned);
    out.airspeed = airdata.airspeed;
}

} // namespace

int main(int argc, char **argv)
{
    Options opts;
    bool want_help = false;
    if (!parse_args(argc, argv, opts, want_help))
    {
        usage(argv[0]);
        return 1;
    }
    if (want_help)
    {
        usage(argv[0]);
        return 0;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    UavModel uav(opts.model);
    try
    {
        auto config = loadModelConfig(opts.model, opts.models_dir);
        uav.initialize(config);
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "last_letter_ardupilot: failed to build model '%s': %s\n",
                opts.model.c_str(), e.what());
        return 1;
    }

    if (opts.want_log)
    {
        // Must precede the first step() so the channels register with the sink.
        uav.enable_logging(opts.log_path);
        if (opts.log_path.empty())
        {
            // logging::enable() names the file after the current date and time.
            printf("Recording to a timestamped .mcap in the working directory\n");
        }
        else
        {
            printf("Recording to %s\n", opts.log_path.c_str());
        }
    }

    JsonConnector connector;
    if (!connector.open(opts.address.c_str(), opts.port))
    {
        // open() reports which step failed and why; errno would be stale by now.
        return 1;
    }
    printf("Simulating '%s', listening for ArduPilot on %s:%u\n",
           opts.model.c_str(), opts.address.c_str(), unsigned(opts.port));
    fflush(stdout);

    double sim_time = 0;
    uint16_t last_frame_rate = 0;
    bool announced_peer = false;
    bool servos_live = false;
    int exit_code = 0;

    try
    {
        while (!g_stop)
        {
            ServoFrame frame;
            if (!connector.receive_servos(frame))
            {
                continue; // Timed out waiting for ArduPilot; keep waiting.
            }

            if (!announced_peer)
            {
                printf("ArduPilot connected from %s\n", connector.peer().c_str());
                fflush(stdout);
                announced_peer = true;
            }

            if (frame.kind == FrameKind::Duplicate)
            {
                // ArduPilot never saw our previous answer. Repeat it rather than
                // stepping, so its clock and ours stay aligned.
                connector.resend_last_state();
                continue;
            }

            if (frame.kind == FrameKind::Reset)
            {
                printf("ArduPilot restarted, resetting the model\n");
                fflush(stdout);
                uav.init();
                sim_time = 0;
            }
            else if (frame.kind == FrameKind::Skipped)
            {
                fprintf(stderr, "last_letter_ardupilot: dropped servo frames near %u\n",
                        frame.frame_count);
            }

            // Adapt to the timestep ArduPilot requests.
            if (frame.frame_rate != last_frame_rate && frame.frame_rate > 0)
            {
                uav.set_dt(1.0 / double(frame.frame_rate));
                last_frame_rate = frame.frame_rate;
            }

            // ArduPilot holds every servo output at 0 for the first couple of
            // seconds after boot, and leaves unassigned channels there for good.
            // Those are not valid servo commands:
            // Only ever process channels that are actually driven, and
            // leave the model on its configured initial input until the first one
            // shows up.
            // Rebuilt from scratch every frame, so nothing ArduPilot commanded
            // earlier can linger on a channel it has stopped driving. Channels it
            // never drives keep InputPwm_t's 1000 us default.
            InputPwm_t input; // 32 channels at 1000 us.

            const size_t channels = std::min(frame.num_channels, input.value.size());
            bool frame_is_driven = false;
            for (size_t i = 0; i < channels; i++)
            {
                if (is_driven(frame.pwm[i]))
                {
                    input.value[i] = frame.pwm[i];
                    frame_is_driven = true;
                }
            }
            if (frame_is_driven)
            {
                if (!servos_live)
                {
                    printf("ArduPilot servo outputs are live\n");
                    fflush(stdout);
                    servos_live = true;
                }
                uav.setInputPwm(input);
            }

            uav.step();
            sim_time += uav.dt;

            StateFrame state;
            fill_state(uav, sim_time, state);
            connector.send_state(state);
        }
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "last_letter_ardupilot: simulation failed at t=%.3f s: %s\n",
                sim_time, e.what());
        exit_code = 1;
    }

    if (opts.want_log)
    {
        // Flushes and closes the MCAP; skipping would truncate the tail of the log.
        uav.disable_logging();
    }
    printf("last_letter_ardupilot: stopped after %.3f s of simulated time\n", sim_time);
    return exit_code;
}
