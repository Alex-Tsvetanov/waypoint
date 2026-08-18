#include "check.hpp"
#include "waypoint/neighbor.hpp"

using namespace waypoint;
using S = NeighborState;
using E = NeighborEvent;
using A = NeighborAction;

WP_TEST(neighbor, hello_from_nothing_reaches_init) {
  const Transition t = neighbor_transition(S::Down, E::HelloReceived);
  CHECK_TRUE(t.next == S::Init);
  CHECK_TRUE(t.action == A::None);
  // A repeated hello is not a state change; only the dead timer is refreshed.
  CHECK_TRUE(neighbor_transition(S::Init, E::HelloReceived).next == S::Init);
}

WP_TEST(neighbor, two_way_starts_the_database_exchange) {
  const Transition t = neighbor_transition(S::Init, E::TwoWayReceived);
  CHECK_TRUE(t.next == S::ExStart);
  CHECK_TRUE(t.action == A::SendDbDescription);
}

WP_TEST(neighbor, the_full_path_runs_in_order) {
  S s = S::Down;
  const E sequence[] = {E::HelloReceived, E::TwoWayReceived, E::NegotiationDone,
                        E::ExchangeDone, E::LoadingDone};
  const S expected[] = {S::Init, S::ExStart, S::Exchange, S::Loading, S::Full};
  for (int i = 0; i < 5; ++i) {
    const Transition t = neighbor_transition(s, sequence[i]);
    s = t.next;
    CHECK_TRUE(s == expected[i]);
  }
  CHECK_TRUE(neighbor_transition(S::Loading, E::LoadingDone).action == A::AdjacencyUp);
}

WP_TEST(neighbor, out_of_order_events_are_ignored) {
  // Loading cannot be reached without going through Exchange first.
  CHECK_TRUE(neighbor_transition(S::Init, E::LoadingDone).next == S::Init);
  CHECK_TRUE(neighbor_transition(S::Down, E::ExchangeDone).next == S::Down);
  CHECK_TRUE(neighbor_transition(S::Full, E::NegotiationDone).next == S::Full);
  CHECK_TRUE(neighbor_transition(S::ExStart, E::ExchangeDone).next == S::ExStart);
}

WP_TEST(neighbor, inactivity_tears_the_adjacency_down_from_any_state) {
  const S states[] = {S::Init, S::TwoWay, S::ExStart, S::Exchange, S::Loading, S::Full};
  for (const S s : states) {
    const Transition t = neighbor_transition(s, E::InactivityTimer);
    CHECK_TRUE(t.next == S::Down);
    CHECK_TRUE(t.action == A::ClearAdjacency);
  }
  // Already down means there is nothing to clear.
  CHECK_TRUE(neighbor_transition(S::Down, E::InactivityTimer).action == A::None);
}

WP_TEST(neighbor, losing_the_reverse_direction_falls_back_to_init) {
  // This is the path a one way link failure takes: the far end still hears us
  // and keeps sending hellos, but stops listing us in them.
  const Transition t = neighbor_transition(S::Full, E::OneWayReceived);
  CHECK_TRUE(t.next == S::Init);
  CHECK_TRUE(t.action == A::ClearAdjacency);
  CHECK_TRUE(neighbor_transition(S::Exchange, E::OneWayReceived).next == S::Init);
  // Below TwoWay there is nothing above Init to lose.
  CHECK_TRUE(neighbor_transition(S::Init, E::OneWayReceived).next == S::Init);
  CHECK_TRUE(neighbor_transition(S::Init, E::OneWayReceived).action == A::None);
  CHECK_TRUE(neighbor_transition(S::Down, E::OneWayReceived).next == S::Down);
}

WP_TEST(neighbor, kill_neighbor_is_immediate) {
  CHECK_TRUE(neighbor_transition(S::Full, E::KillNeighbor) ==
             (Transition{S::Down, A::ClearAdjacency}));
}

WP_TEST(neighbor, two_way_resting_state_can_restart_the_exchange) {
  const Transition t = neighbor_transition(S::TwoWay, E::TwoWayReceived);
  CHECK_TRUE(t.next == S::ExStart);
  CHECK_TRUE(t.action == A::SendDbDescription);
}

WP_TEST(neighbor, every_state_and_event_pair_is_defined) {
  // The table is total: no combination may leave the state machine without an
  // answer, and no answer may move backwards other than the two defined
  // teardown paths.
  const S states[] = {S::Down, S::Init,    S::TwoWay, S::ExStart,
                      S::Exchange, S::Loading, S::Full};
  const E events[] = {E::HelloReceived,   E::TwoWayReceived, E::OneWayReceived,
                      E::NegotiationDone, E::ExchangeDone,   E::LoadingDone,
                      E::InactivityTimer, E::KillNeighbor};
  for (const S s : states) {
    for (const E e : events) {
      const Transition t = neighbor_transition(s, e);
      const bool teardown = e == E::InactivityTimer || e == E::KillNeighbor ||
                            e == E::OneWayReceived;
      if (!teardown) CHECK_TRUE(t.next >= s);
      if (t.next == s) CHECK_TRUE(t.action == A::None || s == S::Init);
      CHECK_TRUE(to_string(t.next) != nullptr);
      CHECK_TRUE(to_string(e) != nullptr);
    }
  }
}
