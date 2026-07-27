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
  Minimal blocking-with-timeout UDP socket, adapted from ArduPilot's
  libraries/SITL/examples/JSON/C++/SocketExample.{h,cpp} (also GPLv3).

  Only the pieces the ArduPilot JSON bridge needs are kept: bind, recv with a
  timeout, sendto, and the address of the peer that sent the last datagram.
  That last one is not optional: ArduPilot sends its servo packets from an
  ephemeral source port and never binds a fixed one, so the reply address can
  only be learned from an incoming packet.
*/
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <netinet/in.h>

namespace last_letter_lib
{
namespace ardupilot
{

class UdpSocket
{
public:
    UdpSocket();
    ~UdpSocket();

    UdpSocket(const UdpSocket &) = delete;
    UdpSocket &operator=(const UdpSocket &) = delete;

    bool is_open() const { return fd_ != -1; }

    bool bind(const char *address, uint16_t port);
    bool reuseaddress() const;
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
    struct sockaddr_in peer_
    {
    };
    bool have_peer_{false};
};

} // namespace ardupilot
} // namespace last_letter_lib
