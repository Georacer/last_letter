/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  last_letter_ardupilot: drives a last_letter UavModel from ArduPilot SITL over
  the JSON backend protocol.

  ArduPilot's SITL::last_letter backend launches this program and then blocks on
  its UDP socket every frame, so the pair runs in lock-step: exactly one
  simulation step is taken per servo packet received.
*/

#include <algorithm>
#include <atomic>
#include <cerrno>
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
using last_letter_lib::ardupilot::JsonConnector;
using last_letter_lib::ardupilot::ServoFrame;
using last_letter_lib::ardupilot::StateFrame;
using last_letter_lib::programming_utils::loadModelConfig;
using last_letter_lib::uav_utils::Airdata;
using last_letter_lib::uav_utils::InputPwm_t;

namespace
{

std::atomic<bool> g_stop{false};

void handle_signal(int /*signum*/)
{
    g_stop = true;
}

void usage(const char *argv0)
{
    printf("usage: %s [options]\n"
           "  --model <name>       aircraft model to simulate (default: skywalker_2013)\n"
           "  --port <n>           UDP port to listen on (default: 9002)\n"
           "  --address <ip>       address to bind to (default: 0.0.0.0)\n"
           "  --models-dir <path>  models root (default: $HOME/last_letter_models)\n"
           "  --log <file.mcap>    record the run to an MCAP log\n"
           "  --help\n",
           argv0);
}

struct Options
{
    std::string model{"skywalker_2013"};
    std::string address{"0.0.0.0"};
    std::string models_dir;
    std::string log_path;
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
        if (i + 1 >= argc)
        {
            fprintf(stderr, "missing value for %s\n", arg.c_str());
            return false;
        }
        const std::string value{argv[++i]};
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
        else if (arg == "--log")
        {
            opts.log_path = value;
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
//    gravity contribution has to come out. state.acceleration.linear cannot be
//    used directly: it is speedDot = F_total/m - w x v, which still includes
//    gravity. Taking the wrench sum minus the gravity wrench and dividing by the
//    mass gives the specific force straight away, and uses whichever gravity
//    model the aircraft is configured with rather than a hardcoded 9.81.
// Takes a mutable reference only because WrenchSum_t::sum() is not const.
void fill_state(UavModel &uav, double timestamp, StateFrame &out)
{
    const auto &state = uav.state;

    out.timestamp = timestamp;

    out.gyro[0] = state.velocity.angular.x();
    out.gyro[1] = state.velocity.angular.y();
    out.gyro[2] = state.velocity.angular.z();

    auto &wrenches = uav.dynamics.wrench_sum;
    // No `auto` here: an Eigen expression template would dangle past the temporary.
    const Eigen::Vector3d specific_force =
        (wrenches.sum().force - wrenches.wrenchGrav.force) / uav.dynamics.inertial.mass;
    out.accel_body[0] = specific_force.x();
    out.accel_body[1] = specific_force.y();
    out.accel_body[2] = specific_force.z();

    out.position[0] = state.pose.position.x();
    out.position[1] = state.pose.position.y();
    out.position[2] = state.pose.position.z();

    // ArduPilot's q1..q4 are w, x, y, z of the body-to-earth rotation, which is
    // exactly what pose.orientation holds.
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
        // Channel 9 is the model's simulation-reset channel by default. ArduPilot
        // drives SERVO10 with whatever function it is assigned, and a value above
        // 1600 us would silently re-initialise the aircraft mid-flight.
        config.set("init/chanReset", -1, false);
        uav.initialize(config);
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "last_letter_ardupilot: failed to build model '%s': %s\n",
                opts.model.c_str(), e.what());
        return 1;
    }

    if (!opts.log_path.empty())
    {
        // Must precede the first step() so the channels register with the sink.
        uav.enable_logging(opts.log_path);
        printf("Recording to %s\n", opts.log_path.c_str());
    }

    JsonConnector connector;
    if (!connector.open(opts.address.c_str(), opts.port))
    {
        fprintf(stderr, "last_letter_ardupilot: could not bind %s:%u: %s\n",
                opts.address.c_str(), unsigned(opts.port), strerror(errno));
        return 1;
    }
    printf("Simulating '%s', listening for ArduPilot on %s:%u\n",
           opts.model.c_str(), opts.address.c_str(), unsigned(opts.port));
    fflush(stdout);

    double sim_time = 0;
    uint16_t last_frame_rate = 0;
    bool announced_peer = false;
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

            // ArduPilot asks for a step size through frame_rate (SIM_RATE_HZ).
            // set_dt() retunes the integrator in place; initialize() would rebuild
            // every sub-model and reset the state.
            if (frame.frame_rate != last_frame_rate && frame.frame_rate > 0)
            {
                uav.set_dt(1.0 / double(frame.frame_rate));
                last_frame_rate = frame.frame_rate;
            }

            InputPwm_t input; // 32 channels, defaulted to 1000 us.
            const size_t channels = std::min(frame.num_channels, input.value.size());
            for (size_t i = 0; i < channels; i++)
            {
                input.value[i] = frame.pwm[i];
            }
            uav.setInputPwm(input);

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

    if (!opts.log_path.empty())
    {
        // Flushes and closes the MCAP; skipping it truncates the tail of the log.
        uav.disable_logging();
    }
    printf("last_letter_ardupilot: stopped after %.3f s of simulated time\n", sim_time);
    return exit_code;
}
