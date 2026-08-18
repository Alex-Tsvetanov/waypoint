// Core scalar types shared by the protocol core, the simulator and the daemon.
#pragma once

#include <cstdint>
#include <limits>
#include <vector>

namespace waypoint {

using NodeId = std::uint32_t;
using Cost = std::uint32_t;

// Monotonic time in microseconds. The protocol core never asks where it came
// from: under the simulator it is virtual time, under the daemon it is the
// monotonic system clock.
using Micros = std::int64_t;

inline constexpr Micros kMs = 1000;
inline constexpr Micros kSec = 1000 * kMs;

inline constexpr Cost kInfCost = std::numeric_limits<Cost>::max();
inline constexpr NodeId kNoNode = std::numeric_limits<NodeId>::max();

// Age at which a link state advertisement is considered expired. RFC 2328
// names 3600 s; it is a compile time constant here so that experiments which
// need faster expiry can rebuild rather than reconfigure at runtime.
inline constexpr std::uint16_t kMaxAgeSec = 3600;

struct Adjacency {
  NodeId peer = kNoNode;
  Cost cost = 1;

  friend bool operator==(const Adjacency&, const Adjacency&) = default;
};

using AdjacencyList = std::vector<Adjacency>;

}  // namespace waypoint
