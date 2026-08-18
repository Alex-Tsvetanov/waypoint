// Live mode: the same protocol core over real UDP sockets.
//
// Everything platform specific is behind these two classes. The router does
// not know a socket exists, and this file contains no protocol logic, so
// neither side can quietly grow a dependency on the other. Winsock and BSD
// sockets differ in the handle type, the shutdown call and the error
// reporting; those three differences are the whole of the porting layer.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "waypoint/env.hpp"
#include "waypoint/router.hpp"

namespace waypoint {

struct Endpoint {
  std::string host = "127.0.0.1";  // IPv4 literal
  std::uint16_t port = 0;
};

class UdpSocket {
 public:
  UdpSocket();
  ~UdpSocket();
  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  // Binds to `port` on all interfaces. Port zero asks the system to choose,
  // which is what the tests use so that two sockets never collide.
  bool open(std::uint16_t port, std::string& error);
  void close();
  bool is_open() const;
  std::uint16_t local_port() const { return local_port_; }

  bool send_to(const Endpoint& to, ByteView datagram, std::string& error);

  enum class Receive { Datagram, TimedOut, Error };
  // Waits at most `timeout` microseconds. A negative timeout blocks.
  Receive receive(Bytes& out, Micros timeout);

 private:
  // Deliberately not a socket handle type: including the platform headers here
  // would put Winsock in the include path of every file that touches a router.
  std::intptr_t handle_;
  std::uint16_t local_port_ = 0;
};

// The Env a live router runs on: a monotonic clock, a timer queue and one
// socket.
class UdpEnv final : public Env {
 public:
  UdpEnv(std::map<NodeId, Endpoint> peers, UdpSocket& socket);

  Micros now() const override;
  void schedule(Micros delay, std::function<void()> fn) override;
  void send(NodeId self, NodeId peer, const Bytes& datagram) override;
  std::uint32_t random(std::uint32_t bound) override;
  void log(const LogEvent& event) override;

  // Drives the router for `duration` of real time: waits for whichever comes
  // first, an incoming datagram or the next timer, and dispatches it. This is
  // the whole event loop, and it is the same shape on every platform.
  void pump(Router& router, Micros duration);

  const std::vector<LogEvent>& events() const { return events_; }
  std::uint64_t datagrams_sent() const { return sent_; }
  std::uint64_t datagrams_received() const { return received_; }
  std::uint64_t send_failures() const { return send_failures_; }

 private:
  std::map<NodeId, Endpoint> peers_;
  UdpSocket& socket_;
  std::map<std::pair<Micros, std::uint64_t>, std::function<void()>> timers_;
  std::uint64_t sequence_ = 0;
  std::uint64_t random_state_;
  std::vector<LogEvent> events_;
  std::uint64_t sent_ = 0;
  std::uint64_t received_ = 0;
  std::uint64_t send_failures_ = 0;
};

}  // namespace waypoint
