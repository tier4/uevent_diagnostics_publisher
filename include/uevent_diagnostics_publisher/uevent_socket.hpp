// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UEVENT_DIAGNOSTICS_PUBLISHER__UEVENT_UEVENT_SOCKET_HPP_
#define UEVENT_DIAGNOSTICS_PUBLISHER__UEVENT_UEVENT_SOCKET_HPP_

#include "uevent_diagnostics_publisher/using_types.hpp"

#include <linux/netlink.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

namespace uevent_diagnostics_publisher
{
namespace util
{
namespace
{
void format_(std::ostringstream &)
{
}

template <class Head, class... Tail>
static void format_(std::ostringstream & os, Head && head, Tail &&... tail)
{
  os << head;
  format_(os, std::forward<Tail>(tail)...);
}
}  // namespace

/* C++20 std::format lesser alternative */
template <class... Args>
std::string format(Args... args)
{
  std::ostringstream os;
  format_(os, args...);
  return os.str();
}

/* C++23 std::to_underlying implementation */
template <class Enum>
constexpr decltype(auto) to_underlying(Enum e) noexcept
{
  return static_cast<std::underlying_type_t<Enum>>(e);
}
}  // namespace util

class UeventSocket
{
public:
  static constexpr int RCVBUF = 2 * 1024 * 1024;

  enum class netlink_group : decltype(sockaddr_nl::nl_groups) { kernel = 1, udev = 2 };

  UeventSocket()
  {
    this->sockfd = socket(AF_NETLINK, SOCK_RAW, NETLINK_KOBJECT_UEVENT);
    if (this->sockfd < 0) {
      throw std::system_error(errno, std::generic_category());
    }

    const int optval = RCVBUF;
    setsockopt(this->sockfd, SOL_SOCKET, SO_RCVBUF, &optval, sizeof(optval));
    setsockopt(this->sockfd, SOL_SOCKET, SO_RCVBUFFORCE, &optval, sizeof(optval));

    struct sockaddr_nl src_addr;
    src_addr.nl_family = AF_NETLINK;
    src_addr.nl_pid = static_cast<decltype(src_addr.nl_pid)>(getpid());
    src_addr.nl_groups = std::numeric_limits<decltype(src_addr.nl_groups)>::max();

    if (
      bind(this->sockfd, reinterpret_cast<const struct sockaddr *>(&src_addr), sizeof(src_addr)) <
      0) {
      close(this->sockfd);
      throw std::system_error(errno, std::generic_category());
    }
  }

  ~UeventSocket() { close(this->sockfd); }

  [[maybe_unused]] void sendUdevEvent(const char * payload, const socklen_t payload_size) const
  {
    struct sockaddr_nl dest_addr;
    dest_addr.nl_family = AF_NETLINK;
    dest_addr.nl_pid = 0;
    dest_addr.nl_groups = util::to_underlying(netlink_group::udev);

    struct iovec iov;
    iov.iov_base = (void *)payload;
    iov.iov_len = payload_size;

    struct msghdr msg_hdr;
    msg_hdr.msg_name = reinterpret_cast<void *>(&dest_addr);
    msg_hdr.msg_namelen = sizeof(dest_addr);
    msg_hdr.msg_iov = &iov;
    msg_hdr.msg_iovlen = 1;
    msg_hdr.msg_control = nullptr;
    msg_hdr.msg_controllen = 0;
    msg_hdr.msg_flags = 0;

    const int bytes_written = sendmsg(this->sockfd, &msg_hdr, 0);
    if (bytes_written < 0) {
      throw std::system_error(errno, std::generic_category());
    }

    if (static_cast<socklen_t>(bytes_written) != payload_size) {
      throw std::system_error(
        EIO, std::generic_category(),
        util::format("failed to send the whole message: ", bytes_written, " bytes written"));
    }
  }

  static decltype(auto) createEnvMap(const UeventRawDataType & uevent_raw_data)
  {
    const std::vector<char> raw_uevent = uevent_raw_data.first;

    size_t size = uevent_raw_data.second;
    UeventDataType env_list{};

    auto current = raw_uevent.begin();
    const auto end = raw_uevent.begin() + size + 1;
    decltype(current) terminator;

    for (; current != end; current = terminator + 1) {
      terminator = std::find(current, end, '\0');
      const auto equal = std::find(current, terminator, '=');
      if (equal == terminator) continue;

      auto key = std::string(current, equal);
      auto value = std::string(equal + 1, terminator);
      env_list[key] = value;
    }

    return env_list;
  }

  decltype(auto) receiveUeventData()
  {
    constexpr int BUFFER_SIZE = 16 * 1024;
    std::vector<char> buffer(BUFFER_SIZE);

    // Since AF_NETLINK is datagram oriented, if larger data than the buffer is received,
    // the excess bytes are discarded immediatly (repeated recv call does not make sense)
    const auto bytes_read = recv(this->sockfd, buffer.data(), BUFFER_SIZE - 1, 0);
    if (bytes_read <= 0) {
      // receive failed
      throw std::system_error(errno, std::generic_category());
    }

    buffer[bytes_read] = '\0';

    return std::make_pair(std::move(buffer), bytes_read);
  }

private:
  int sockfd;

};  // class UeventSocket
}  // namespace uevent_diagnostics_publisher

#endif  // UEVENT_DIAGNOSTICS_PUBLISHER__UEVENT_UEVENT_SOCKET_HPP_
