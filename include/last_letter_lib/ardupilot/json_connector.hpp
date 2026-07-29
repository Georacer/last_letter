/*
  ArduPilot SITL "JSON" backend protocol, physics-engine side.

  ArduPilot sends a binary servo packet over UDP and blocks until the physics
  engine answers with a newline-delimited JSON state frame; that blocking read is
  what makes the pair run in lock-step. See
  ardupilot/libraries/SITL/examples/JSON/readme.md and
  ardupilot/libraries/SITL/SIM_JSON.cpp.

  This class is a utility driver to exchange ServoFrame's and StateFrame's.
*/
#pragma once

#include <cstdint>
#include <string>

#include "last_letter_lib/ardupilot/udp_socket.hpp"

namespace last_letter_lib {
namespace ardupilot {

// Maximum servo channels ArduPilot can emit (with SERVO_32_ENABLE = 1).
constexpr size_t max_servo_channels{32};

// Plausible bounds for a servo command. The protocol documents PWM as "typically
// in the 1000 to 2000 range"; the margin allows for extended endpoints. Anything
// outside means the channel is not being driven at all - notably, ArduPilot emits
// 0 on every channel for the first seconds after boot, and permanently on
// channels with no output function assigned.
constexpr uint16_t min_valid_pwm{800};
constexpr uint16_t max_valid_pwm{2200};

inline bool is_driven(uint16_t pwm)
{
    return pwm >= min_valid_pwm && pwm <= max_valid_pwm;
}

// How a freshly received packet relates to the previous one.
enum class FrameKind
{
    Normal,    // frame_count advanced as expected: run a simulation step.
    Skipped,   // frame_count jumped: packets were lost, still run a step.
    Duplicate, // frame_count repeated: ArduPilot re-sent after a timeout. Re-send
               // the last state instead of stepping, so sim time stays in lock-step.
    Reset,     // frame_count went backwards: ArduPilot restarted, re-init the model.
};

struct ServoFrame
{
    uint16_t frame_rate{0};  // Requested simulation rate, Hz (SIM_RATE_HZ).
    uint32_t frame_count{0}; // Increments per ArduPilot output frame.
    size_t num_channels{0};  // 16 or 32, depending on the packet magic.
    uint16_t pwm[max_servo_channels]{};
    FrameKind kind{FrameKind::Normal};
};

// The JSON payload sent back to ArduPilot. All vectors follow the protocol's
// conventions: body frame (FRD) for the IMU, NED earth frame for the rest.
struct StateFrame
{
    double timestamp{0};        // Absolute physics time, s (not a delta).
    double gyro[3]{};           // Body-frame angular rates, rad/s.
    double accel_body[3]{};     // Body-frame specific force, m/s^2 (~[0,0,-9.81] at rest).
    double position[3]{};       // NED position relative to home, m.
    double quaternion[4]{1, 0, 0, 0}; // Body-to-earth rotation, [w, x, y, z].
    double velocity[3]{};       // NED velocity, m/s.
    double airspeed{0};         // True airspeed, m/s.
    double velocity_wind[3]{};  // NED wind vector, m/s.
};

class JsonConnector
{
public:
    // Bind the UDP port ArduPilot sends servo packets to (9002 by default, or
    // 9002 + instance*10 when SITL runs with -I<n>). Replies always go back to
    // the source address of the last packet, so no target needs configuring.
    bool open(const char *bind_address, uint16_t port);

    // Wait for one servo packet. Returns false on timeout or on a packet that is
    // not a valid ArduPilot servo frame; the caller is expected to loop.
    bool receive_servos(ServoFrame &frame, uint32_t timeout_ms = 100);

    // Serialise and send a state frame to ArduPilot. Returns false if the frame
    // contains non-finite values or the datagram could not be sent.
    bool send_state(const StateFrame &state);

    // Re-send the last frame passed to send_state(). Used to answer a duplicate
    // servo packet without advancing the simulation.
    bool resend_last_state();

    const std::string &peer() const { return peer_; }

private:
    UdpSocket sock_;
    std::string peer_;

    uint32_t last_frame_count_{0};
    bool have_frame_count_{false};

    // The serialised JSON of the most recent send_state(), kept for re-sends.
    char buffer_[1024]{}; // 21 fields take roughly 500B.
    size_t buffer_len_{0};
};

} // namespace ardupilot
} // namespace last_letter_lib
