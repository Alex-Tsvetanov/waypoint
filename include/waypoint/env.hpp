// The boundary the protocol core is not allowed to cross.
//
// Everything below this interface is either a virtual clock with an event
// queue, or a monotonic clock with real sockets. The protocol core sees only
// `Env`, which is why one build of the routing logic serves both the simulator
// and the daemon. The boundary is structural rather than enforced by the
// linker: the platform socket headers are included in src/udp.cpp and in no
// other file, so a violation is one new include away from being visible.
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "waypoint/packet.hpp"
#include "waypoint/types.hpp"

namespace waypoint {

enum class EventKind : std::uint8_t {
  NeighborState,   // a = previous state, b = new state
  LsaOriginated,   // peer = origin (self), a = sequence number
  LsaInstalled,    // peer = origin, a = sequence number
  LsaReflooded,    // peer = origin, a = number of neighbours it went to
  SpfComputed,     // a = index of the computation at this node
  RouteInstalled,  // a = number of changed destinations
  LinkFailure,     // injected: peer = far end, a = 1 one way, 0 both ways
  LinkRestore,     // injected
  NodeFailure,     // injected
  NodeRestore,     // injected
};

const char* to_string(EventKind kind);

struct LogEvent {
  Micros time = 0;
  NodeId node = kNoNode;
  EventKind kind = EventKind::SpfComputed;
  NodeId peer = kNoNode;
  std::uint32_t a = 0;
  std::uint32_t b = 0;
};

class Env {
 public:
  virtual ~Env() = default;

  // Current time in microseconds. Virtual under the simulator, monotonic
  // under the daemon.
  virtual Micros now() const = 0;

  // One shot timer. There is deliberately no cancel operation: callbacks that
  // have been superseded compare a generation stamp and return. That keeps the
  // interface small and removes a whole class of dangling handle.
  virtual void schedule(Micros delay, std::function<void()> fn) = 0;

  // Hands a complete datagram to the layer below. Whether it becomes a queued
  // simulator event or a `sendto` call is not the core's business.
  virtual void send(NodeId self, NodeId peer, const Bytes& datagram) = 0;

  // Seeded under the simulator so that timer jitter is reproducible.
  virtual std::uint32_t random(std::uint32_t bound) = 0;

  virtual void log(const LogEvent& event) { (void)event; }
};

}  // namespace waypoint
