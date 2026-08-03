#include "udp_socket.hpp"

#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace last_letter_lib {
namespace ardupilot {

namespace {

void make_sockaddr(const char *address, uint16_t port, struct sockaddr_in &sockaddr)
{
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_port = htons(port);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(address);
}

} // namespace

UdpSocket::UdpSocket() : fd_(socket(AF_INET, SOCK_DGRAM, 0))
{
    if (fd_ != -1)
    {
        fcntl(fd_, F_SETFD, FD_CLOEXEC);
    }
}

UdpSocket::~UdpSocket()
{
    if (fd_ != -1)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool UdpSocket::bind(const char *address, uint16_t port)
{
    sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);
    return ::bind(fd_, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) == 0;
}

bool UdpSocket::set_blocking(bool blocking) const
{
    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags == -1)
    {
        return false;
    }
    if (blocking)
    {
        flags &= ~O_NONBLOCK;
    }
    else
    {
        flags |= O_NONBLOCK;
    }
    return fcntl(fd_, F_SETFL, flags) != -1;
}

bool UdpSocket::pollin(uint32_t timeout_ms) const
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd_, &fds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;

    return select(fd_ + 1, &fds, nullptr, nullptr, &tv) == 1;
}

ssize_t UdpSocket::recv(void *buf, size_t size, uint32_t timeout_ms)
{
    if (!pollin(timeout_ms))
    {
        return -1;
    }
    sockaddr_in from;
    socklen_t len = sizeof(from);
    const ssize_t ret = ::recvfrom(fd_, buf, size, MSG_DONTWAIT, (struct sockaddr *)&from, &len);
    if (ret >= 0)
    {
        peer_ = from;
        have_peer_ = true;
    }
    return ret;
}

ssize_t UdpSocket::sendto(const void *buf, size_t size, const char *address, uint16_t port)
{
    sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);
    return ::sendto(fd_, buf, size, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}

ssize_t UdpSocket::reply(const void *buf, size_t size)
{
    if (!have_peer_)
    {
        return -1;
    }
    return ::sendto(fd_, buf, size, 0, (struct sockaddr *)&peer_, sizeof(peer_));
}

std::string UdpSocket::peer_str() const
{
    if (!have_peer_)
    {
        return std::string();
    }
    // inet_ntop instead of inet_ntoa: the latter returns a pointer into a static
    // buffer that the next call overwrites.
    char ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &peer_.sin_addr, ip, sizeof(ip));
    return std::string(ip) + ":" + std::to_string(ntohs(peer_.sin_port));
}

} // namespace ardupilot
} // namespace last_letter_lib
