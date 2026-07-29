/*
  Minimal blocking-with-timeout UDP socket, adapted from ArduPilot's
  libraries/SITL/examples/JSON/C++/SocketExample.{h,cpp}.
*/
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <netinet/in.h>

namespace last_letter_lib {
namespace ardupilot {

class UdpSocket
{
public:
    UdpSocket();
    ~UdpSocket();

    UdpSocket(const UdpSocket &) = delete;
    UdpSocket &operator=(const UdpSocket &) = delete;

    bool is_open() const { return fd_ != -1; }

    bool bind(const char *address, uint16_t port);
    bool set_blocking(bool blocking) const;

    // Wait up to timeout_ms for a datagram. Returns the number of bytes read, or
    // -1 on timeout/error. Records the sender so reply_to() can be used.
    ssize_t recv(void *buf, size_t size, uint32_t timeout_ms);

    // Send to an explicit destination.
    ssize_t sendto(const void *buf, size_t size, const char *address, uint16_t port);

    // Send to whoever sent the last datagram received. Returns -1 if nothing has
    // been received yet.
    ssize_t reply(const void *buf, size_t size);

    // Human-readable "ip:port" of the last sender, or an empty string.
    std::string peer_str() const;

    bool have_peer() const { return have_peer_; }

private:
    bool pollin(uint32_t timeout_ms) const;

    int fd_{-1};
    sockaddr_in peer_{};
    bool have_peer_{false};
};

} // namespace ardupilot
} // namespace last_letter_lib
