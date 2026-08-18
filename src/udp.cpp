#include "waypoint/udp.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <utility>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using SocketHandle = SOCKET;
#else
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using SocketHandle = int;
#endif

namespace waypoint {
namespace {

#ifdef _WIN32
constexpr std::intptr_t kInvalidHandle = static_cast<std::intptr_t>(INVALID_SOCKET);

// Winsock has to be started once per process and stopped at exit. A function
// local static gives exactly that, with no order of initialisation problem.
struct WinsockGuard {
  WinsockGuard() {
    WSADATA data;
    ok = WSAStartup(MAKEWORD(2, 2), &data) == 0;
  }
  ~WinsockGuard() {
    if (ok) WSACleanup();
  }
  bool ok = false;
};

bool ensure_started() {
  static WinsockGuard guard;
  return guard.ok;
}

std::string last_error() { return "winsock error " + std::to_string(WSAGetLastError()); }

void close_handle(std::intptr_t h) { closesocket(static_cast<SocketHandle>(h)); }
#else
constexpr std::intptr_t kInvalidHandle = -1;

bool ensure_started() { return true; }

std::string last_error() { return std::string("errno ") + std::to_string(errno); }

void close_handle(std::intptr_t h) { ::close(static_cast<SocketHandle>(h)); }
#endif

SocketHandle as_socket(std::intptr_t h) { return static_cast<SocketHandle>(h); }

Micros monotonic_now() {
  using clock = std::chrono::steady_clock;
  static const clock::time_point epoch = clock::now();
  return std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - epoch)
      .count();
}

constexpr std::size_t kMaxDatagram = 2048;

}  // namespace

UdpSocket::UdpSocket() : handle_(kInvalidHandle) {}

UdpSocket::~UdpSocket() { close(); }

bool UdpSocket::is_open() const { return handle_ != kInvalidHandle; }

void UdpSocket::close() {
  if (handle_ != kInvalidHandle) {
    close_handle(handle_);
    handle_ = kInvalidHandle;
  }
  local_port_ = 0;
}

bool UdpSocket::open(std::uint16_t port, std::string& error) {
  if (!ensure_started()) {
    error = "socket library failed to start";
    return false;
  }
  close();

  const SocketHandle fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd == static_cast<SocketHandle>(kInvalidHandle)) {
    error = "socket: " + last_error();
    return false;
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(port);
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
    error = "bind: " + last_error();
    close_handle(static_cast<std::intptr_t>(fd));
    return false;
  }

  // Port zero means the system chose; report back which one, so a caller can
  // tell a peer where to reach it.
  sockaddr_in bound{};
#ifdef _WIN32
  int length = static_cast<int>(sizeof(bound));
#else
  socklen_t length = sizeof(bound);
#endif
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &length) == 0) {
    local_port_ = ntohs(bound.sin_port);
  } else {
    local_port_ = port;
  }

  handle_ = static_cast<std::intptr_t>(fd);
  error.clear();
  return true;
}

bool UdpSocket::send_to(const Endpoint& to, ByteView datagram, std::string& error) {
  if (!is_open()) {
    error = "socket is not open";
    return false;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(to.port);
  if (::inet_pton(AF_INET, to.host.c_str(), &address.sin_addr) != 1) {
    error = "not an IPv4 address: " + to.host;
    return false;
  }

  const auto sent = ::sendto(as_socket(handle_),
                             reinterpret_cast<const char*>(datagram.data()),
                             static_cast<int>(datagram.size()), 0,
                             reinterpret_cast<const sockaddr*>(&address),
                             sizeof(address));
  if (sent < 0 || static_cast<std::size_t>(sent) != datagram.size()) {
    error = "sendto: " + last_error();
    return false;
  }
  error.clear();
  return true;
}

UdpSocket::Receive UdpSocket::receive(Bytes& out, Micros timeout) {
  if (!is_open()) return Receive::Error;

  fd_set readable;
  FD_ZERO(&readable);
  FD_SET(as_socket(handle_), &readable);

  timeval tv{};
  timeval* tv_pointer = nullptr;
  if (timeout >= 0) {
    tv.tv_sec = static_cast<decltype(tv.tv_sec)>(timeout / 1000000);
    tv.tv_usec = static_cast<decltype(tv.tv_usec)>(timeout % 1000000);
    tv_pointer = &tv;
  }

#ifdef _WIN32
  const int nfds = 0;  // ignored by Winsock
#else
  const int nfds = static_cast<int>(as_socket(handle_)) + 1;
#endif
  const int ready = ::select(nfds, &readable, nullptr, nullptr, tv_pointer);
  if (ready == 0) return Receive::TimedOut;
  if (ready < 0) return Receive::Error;

  out.assign(kMaxDatagram, 0);
  const auto received = ::recvfrom(as_socket(handle_),
                                   reinterpret_cast<char*>(out.data()),
                                   static_cast<int>(out.size()), 0, nullptr, nullptr);
  if (received <= 0) {
    out.clear();
    return Receive::Error;
  }
  out.resize(static_cast<std::size_t>(received));
  return Receive::Datagram;
}

// --- UdpEnv ----------------------------------------------------------------

UdpEnv::UdpEnv(std::map<NodeId, Endpoint> peers, UdpSocket& socket)
    : peers_(std::move(peers)), socket_(socket) {
  // Seeded from the clock: in live mode the hello offset only has to avoid
  // synchronising the routers, it does not have to be reproducible.
  random_state_ = static_cast<std::uint64_t>(monotonic_now()) * 2654435761ull + 1;
}

Micros UdpEnv::now() const { return monotonic_now(); }

void UdpEnv::schedule(Micros delay, std::function<void()> fn) {
  const Micros when = now() + (delay < 0 ? 0 : delay);
  timers_.emplace(std::pair<Micros, std::uint64_t>{when, sequence_++}, std::move(fn));
}

void UdpEnv::send(NodeId self, NodeId peer, const Bytes& datagram) {
  (void)self;
  const auto it = peers_.find(peer);
  if (it == peers_.end()) {
    ++send_failures_;
    return;
  }
  std::string error;
  if (socket_.send_to(it->second, datagram, error)) {
    ++sent_;
  } else {
    // A failed send is a lost packet, which the protocol already tolerates:
    // hellos repeat and advertisements are retransmitted until acknowledged.
    ++send_failures_;
  }
}

std::uint32_t UdpEnv::random(std::uint32_t bound) {
  if (bound == 0) return 0;
  random_state_ = random_state_ * 6364136223846793005ull + 1442695040888963407ull;
  return static_cast<std::uint32_t>((random_state_ >> 33) % bound);
}

void UdpEnv::log(const LogEvent& event) { events_.push_back(event); }

void UdpEnv::pump(Router& router, Micros duration) {
  const Micros deadline = now() + duration;
  Bytes buffer;

  while (true) {
    const Micros current = now();
    if (current >= deadline) break;

    Micros wake = deadline;
    if (!timers_.empty()) wake = std::min(wake, timers_.begin()->first.first);
    const Micros wait = wake > current ? wake - current : 0;

    if (wait > 0) {
      if (socket_.receive(buffer, wait) == UdpSocket::Receive::Datagram) {
        ++received_;
        router.receive(buffer);
        // Drain whatever else is already queued before going back to sleep.
        while (socket_.receive(buffer, 0) == UdpSocket::Receive::Datagram) {
          ++received_;
          router.receive(buffer);
        }
      }
    }

    while (!timers_.empty() && timers_.begin()->first.first <= now()) {
      const auto it = timers_.begin();
      std::function<void()> fn = std::move(it->second);
      timers_.erase(it);
      fn();
    }
  }
}

}  // namespace waypoint
