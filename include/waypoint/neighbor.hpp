// The adjacency state machine, written as an explicit transition table rather
// than as nested conditionals. A table can be read against RFC 2328 section
// 10.3 line by line, and every entry in it can be given its own test.
#pragma once

#include <cstdint>

namespace waypoint {

enum class NeighborState : std::uint8_t {
  Down = 0,
  Init = 1,      // hello heard, but the far end has not yet listed us
  TwoWay = 2,    // both ends hear each other
  ExStart = 3,   // database exchange beginning
  Exchange = 4,  // database summaries in flight
  Loading = 5,   // missing advertisements requested
  Full = 6,      // adjacency usable, the link may enter our advertisement
};

enum class NeighborEvent : std::uint8_t {
  HelloReceived,    // any hello from the neighbour
  TwoWayReceived,   // a hello that lists us, so the link is confirmed both ways
  OneWayReceived,   // a hello that no longer lists us
  NegotiationDone,  // our database summary has been sent
  ExchangeDone,     // the neighbour's summary has been processed
  LoadingDone,      // every requested advertisement has arrived
  InactivityTimer,  // no hello within the dead interval
  KillNeighbor,     // the interface went away underneath us
};

enum class NeighborAction : std::uint8_t {
  None,
  SendDbDescription,  // enter exchange by advertising what we hold
  SendRequests,       // ask for what the summary showed we lack
  AdjacencyUp,        // the link may now be advertised, recompute routes
  ClearAdjacency,     // forget the neighbour's contribution, recompute routes
};

struct Transition {
  NeighborState next;
  NeighborAction action;

  friend bool operator==(const Transition&, const Transition&) = default;
};

// Pure function of state and event. Combinations with no defined meaning leave
// the state alone and do nothing, which is what RFC 2328 calls an ignored
// event rather than an error.
Transition neighbor_transition(NeighborState state, NeighborEvent event);

const char* to_string(NeighborState state);
const char* to_string(NeighborEvent event);

}  // namespace waypoint
