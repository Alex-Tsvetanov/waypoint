#include "waypoint/neighbor.hpp"

#include "waypoint/env.hpp"

namespace waypoint {

Transition neighbor_transition(NeighborState state, NeighborEvent event) {
  using S = NeighborState;
  using E = NeighborEvent;
  using A = NeighborAction;

  // Two events apply in every state and are handled before the table, exactly
  // as RFC 2328 handles them.
  if (event == E::KillNeighbor || event == E::InactivityTimer) {
    if (state == S::Down) return Transition{S::Down, A::None};
    return Transition{S::Down, A::ClearAdjacency};
  }
  if (event == E::OneWayReceived) {
    // The far end stopped listing us. Everything above Init is void; the
    // hearing itself survives, so we fall back to Init and not to Down.
    if (state <= S::Init) return Transition{state, A::None};
    return Transition{S::Init, A::ClearAdjacency};
  }

  switch (state) {
    case S::Down:
      if (event == E::HelloReceived) return Transition{S::Init, A::None};
      break;

    case S::Init:
      if (event == E::HelloReceived) return Transition{S::Init, A::None};
      if (event == E::TwoWayReceived) {
        return Transition{S::ExStart, A::SendDbDescription};
      }
      break;

    case S::TwoWay:
      // Reached only when an adjacency is torn down while both ends still hear
      // each other. The next confirmed hello restarts the exchange.
      if (event == E::TwoWayReceived) {
        return Transition{S::ExStart, A::SendDbDescription};
      }
      break;

    case S::ExStart:
      if (event == E::NegotiationDone) return Transition{S::Exchange, A::None};
      break;

    case S::Exchange:
      if (event == E::ExchangeDone) return Transition{S::Loading, A::SendRequests};
      break;

    case S::Loading:
      if (event == E::LoadingDone) return Transition{S::Full, A::AdjacencyUp};
      break;

    case S::Full:
      break;
  }
  return Transition{state, A::None};
}

const char* to_string(NeighborState state) {
  switch (state) {
    case NeighborState::Down: return "Down";
    case NeighborState::Init: return "Init";
    case NeighborState::TwoWay: return "TwoWay";
    case NeighborState::ExStart: return "ExStart";
    case NeighborState::Exchange: return "Exchange";
    case NeighborState::Loading: return "Loading";
    case NeighborState::Full: return "Full";
  }
  return "?";
}

const char* to_string(NeighborEvent event) {
  switch (event) {
    case NeighborEvent::HelloReceived: return "HelloReceived";
    case NeighborEvent::TwoWayReceived: return "TwoWayReceived";
    case NeighborEvent::OneWayReceived: return "OneWayReceived";
    case NeighborEvent::NegotiationDone: return "NegotiationDone";
    case NeighborEvent::ExchangeDone: return "ExchangeDone";
    case NeighborEvent::LoadingDone: return "LoadingDone";
    case NeighborEvent::InactivityTimer: return "InactivityTimer";
    case NeighborEvent::KillNeighbor: return "KillNeighbor";
  }
  return "?";
}

const char* to_string(EventKind kind) {
  switch (kind) {
    case EventKind::NeighborState: return "neighbor-state";
    case EventKind::LsaOriginated: return "lsa-originated";
    case EventKind::LsaInstalled: return "lsa-installed";
    case EventKind::LsaReflooded: return "lsa-reflooded";
    case EventKind::SpfComputed: return "spf-computed";
    case EventKind::RouteInstalled: return "route-installed";
    case EventKind::LinkFailure: return "link-failure";
    case EventKind::LinkRestore: return "link-restore";
    case EventKind::NodeFailure: return "node-failure";
    case EventKind::NodeRestore: return "node-restore";
  }
  return "?";
}

}  // namespace waypoint
