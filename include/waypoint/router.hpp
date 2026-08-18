// The protocol core. One instance is one router. It contains no reference to
// a clock, a socket or a thread: everything it needs from the outside world
// arrives through `Env`.
#pragma once

#include <cstdint>
#include <map>
#include <set>

#include "waypoint/env.hpp"
#include "waypoint/lsdb.hpp"
#include "waypoint/neighbor.hpp"
#include "waypoint/spf.hpp"

namespace waypoint {

class Router {
 public:
  struct Config {
    NodeId id = kNoNode;
    Micros hello_interval = 1 * kSec;
    Micros dead_interval = 4 * kSec;
    // Retransmission period for unacknowledged advertisements and for the
    // database summary while an adjacency is still forming.
    Micros rxmt_interval = 1 * kSec;
    // Hold down between the first database change and the route computation,
    // so a burst of advertisements costs one computation and not one each.
    Micros spf_delay = 100 * kMs;
    // Between computing routes and putting them into force. Kept as a separate
    // step because the interval between the two is exactly where a router
    // knows the new topology but still forwards along the old one.
    Micros install_delay = 0;
    // Minimum spacing between two advertisements originated by this router.
    Micros min_lsa_interval = 0;
    Micros lsa_refresh_interval = 1800 * kSec;
  };

  Router(Config config, Env& env);

  // Configured local interfaces. Adding a link does not create an adjacency;
  // the adjacency has to be discovered by hello exchange like any other.
  void add_link(NodeId peer, Cost cost);

  void start();

  // Entry point for an incoming datagram. The sender is read from the header,
  // so this is the same call under the simulator and under a real socket.
  void receive(ByteView datagram);

  NodeId id() const { return config_.id; }
  const RoutingTable& rib() const { return rib_; }
  const Lsdb& lsdb() const { return lsdb_; }
  NeighborState state_of(NodeId peer) const;
  const std::map<NodeId, Cost>& links() const { return links_; }

  std::uint32_t spf_runs() const { return spf_runs_; }
  std::uint32_t originations() const { return originations_; }
  std::uint64_t route_changes() const { return route_changes_; }
  bool malformed_seen() const { return malformed_ > 0; }
  std::uint64_t malformed_count() const { return malformed_; }

 private:
  struct Neighbor {
    Cost cost = 1;
    NeighborState state = NeighborState::Down;
    std::uint64_t dead_gen = 0;
    std::uint64_t dd_gen = 0;
    std::map<NodeId, Lsa> rxmt;  // sent, not yet acknowledged
    std::set<NodeId> pending;    // requested, not yet arrived
    int request_attempts = 0;
  };

  // How often an unanswered link state request is repeated before the
  // adjacency gives up waiting and comes up anyway. A neighbour that offered
  // an advertisement in its summary and then cannot supply it has withdrawn
  // it in the meantime; holding the adjacency down for ever over that would
  // cost more than the missing advertisement, which flooding will deliver
  // again if it still exists.
  static constexpr int kMaxRequestAttempts = 3;

  void send_hello();
  void arm_dead_timer(NodeId peer);
  void arm_dd_retransmit(NodeId peer);
  void arm_retransmit();

  void handle_hello(NodeId from, ByteView datagram);
  void handle_db_description(NodeId from, ByteView datagram);
  void handle_ls_request(NodeId from, ByteView datagram);
  void handle_ls_update(NodeId from, ByteView datagram);
  void handle_ls_ack(NodeId from, ByteView datagram);

  void apply(NodeId peer, NeighborEvent event);
  void send_db_description(NodeId peer, bool reply);
  void process_summary(NodeId peer, const std::vector<LsaHandle>& summary);

  void originate_lsa();
  void flood(const Lsa& lsa, NodeId exclude);
  void schedule_spf();
  void run_spf();
  void install(RoutingTable next);
  void emit(EventKind kind, NodeId peer, std::uint32_t a, std::uint32_t b = 0);

  Config config_;
  Env& env_;
  std::map<NodeId, Cost> links_;
  std::map<NodeId, Neighbor> neighbors_;
  Lsdb lsdb_;
  RoutingTable rib_;

  std::uint32_t own_seq_ = 0;
  Micros last_origination_ = -1;
  bool origination_pending_ = false;
  bool spf_pending_ = false;
  bool rxmt_armed_ = false;
  bool started_ = false;

  std::uint32_t spf_runs_ = 0;
  std::uint32_t originations_ = 0;
  std::uint64_t route_changes_ = 0;
  std::uint64_t malformed_ = 0;
};

}  // namespace waypoint
