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

#include "last_letter_lib/ardupilot/json_connector.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace last_letter_lib
{
namespace ardupilot
{

namespace
{

// Protocol constants from ardupilot/libraries/SITL/SIM_JSON.h.
constexpr uint16_t magic_16_channels{18458};
constexpr uint16_t magic_32_channels{29569};

constexpr size_t servo_header_size{8}; // magic + frame_rate + frame_count
constexpr size_t packet_size_16{servo_header_size + 16 * sizeof(uint16_t)};
constexpr size_t packet_size_32{servo_header_size + 32 * sizeof(uint16_t)};

bool all_finite(const double *v, size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        if (!std::isfinite(v[i]))
        {
            return false;
        }
    }
    return true;
}

} // namespace

bool JsonConnector::open(const char *bind_address, uint16_t port)
{
    if (!sock_.is_open())
    {
        return false;
    }
    sock_.set_blocking(false);
    sock_.reuseaddress();
    return sock_.bind(bind_address, port);
}

bool JsonConnector::receive_servos(ServoFrame &frame, uint32_t timeout_ms)
{
    uint8_t buf[packet_size_32];
    const ssize_t ret = sock_.recv(buf, sizeof(buf), timeout_ms);
    if (ret <= 0)
    {
        return false;
    }

    uint16_t magic = 0;
    memcpy(&magic, buf + 0, sizeof(magic));

    size_t num_channels = 0;
    if (magic == magic_16_channels && (size_t)ret == packet_size_16)
    {
        num_channels = 16;
    }
    else if (magic == magic_32_channels && (size_t)ret == packet_size_32)
    {
        num_channels = 32;
    }
    else
    {
        fprintf(stderr, "last_letter_ardupilot: ignoring %zd-byte packet with magic %u\n",
                ret, unsigned(magic));
        return false;
    }

    peer_ = sock_.peer_str();

    memcpy(&frame.frame_rate, buf + 2, sizeof(frame.frame_rate));
    memcpy(&frame.frame_count, buf + 4, sizeof(frame.frame_count));
    memset(frame.pwm, 0, sizeof(frame.pwm));
    memcpy(frame.pwm, buf + servo_header_size, num_channels * sizeof(uint16_t));
    frame.num_channels = num_channels;

    if (!have_frame_count_)
    {
        frame.kind = FrameKind::Normal;
        have_frame_count_ = true;
    }
    else if (frame.frame_count < last_frame_count_)
    {
        // ArduPilot restarted: its counter is reset with SITL.
        frame.kind = FrameKind::Reset;
    }
    else if (frame.frame_count == last_frame_count_)
    {
        // ArduPilot re-sends the same frame, without incrementing the counter,
        // when it has been waiting on us for ~1.1 s (SIM_JSON.cpp:326).
        frame.kind = FrameKind::Duplicate;
    }
    else if (frame.frame_count != last_frame_count_ + 1)
    {
        frame.kind = FrameKind::Skipped;
    }
    else
    {
        frame.kind = FrameKind::Normal;
    }

    last_frame_count_ = frame.frame_count;
    return true;
}

bool JsonConnector::send_state(const StateFrame &s)
{
    if (!std::isfinite(s.timestamp) || !std::isfinite(s.airspeed) ||
        !all_finite(s.gyro, 3) || !all_finite(s.accel_body, 3) || !all_finite(s.position, 3) ||
        !all_finite(s.quaternion, 4) || !all_finite(s.velocity, 3) || !all_finite(s.velocity_wind, 3))
    {
        fprintf(stderr, "last_letter_ardupilot: refusing to send a non-finite state frame\n");
        return false;
    }

    // ArduPilot parses this with a hand-rolled strstr scanner, not a JSON parser
    // (SIM_JSON.cpp:177). Three properties of the text below are load-bearing:
    //  - the record is both preceded and terminated by '\n', which is how
    //    SIM_JSON::recv_fdm() delimits the newest complete frame;
    //  - there is no whitespace anywhere, because the scanner jumps a fixed
    //    strlen(key)+2 past a key to reach its value;
    //  - "velocity" comes before "velocity_wind", otherwise strstr() would match
    //    the wind key when looking for the velocity one.
    const int written = snprintf(
        buffer_, sizeof(buffer_),
        "\n{"
        "\"timestamp\":%.12g,"
        "\"imu\":{\"gyro\":[%.9g,%.9g,%.9g],\"accel_body\":[%.9g,%.9g,%.9g]},"
        "\"position\":[%.12g,%.12g,%.12g],"
        "\"quaternion\":[%.9g,%.9g,%.9g,%.9g],"
        "\"velocity\":[%.9g,%.9g,%.9g],"
        "\"airspeed\":%.9g,"
        "\"velocity_wind\":[%.9g,%.9g,%.9g]"
        "}\n",
        s.timestamp,
        s.gyro[0], s.gyro[1], s.gyro[2],
        s.accel_body[0], s.accel_body[1], s.accel_body[2],
        s.position[0], s.position[1], s.position[2],
        s.quaternion[0], s.quaternion[1], s.quaternion[2], s.quaternion[3],
        s.velocity[0], s.velocity[1], s.velocity[2],
        s.airspeed,
        s.velocity_wind[0], s.velocity_wind[1], s.velocity_wind[2]);

    if (written < 0 || (size_t)written >= sizeof(buffer_))
    {
        fprintf(stderr, "last_letter_ardupilot: state frame did not fit the send buffer\n");
        buffer_len_ = 0;
        return false;
    }
    buffer_len_ = (size_t)written;

    return resend_last_state();
}

bool JsonConnector::resend_last_state()
{
    if (buffer_len_ == 0)
    {
        return false;
    }
    return sock_.reply(buffer_, buffer_len_) == (ssize_t)buffer_len_;
}

} // namespace ardupilot
} // namespace last_letter_lib
