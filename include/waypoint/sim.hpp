// Deterministic discrete event simulator.
//
// Nothing advances except by taking the next event off the queue, so a run is
// a pure function of the topology, the parameters and the seed. The routers it
// drives are the same `Router` objects the daemon uses; the simulator supplies
// their `Env` and nothing else.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "waypoint/analysis.hpp"
#include "waypoint/env.hpp"
#include "waypoint/router.hpp"
#include "waypoint/topology.hpp"

namespace waypoint {

// Defined in sim.cpp: the Env implementation the simulator hands each router.
class NodeEnv;

struct SimMetrics {
  std::uint64_t messages_sent = 0;
  std::uint64_t bytes_sent = 0;
  std::uint64_t messages_dropped = 0;
  std::uint64_t hello_messages = 0;
  std::uint64_t hello_bytes = 0;
  // Flooding overhead proper: updates, acknowledgements, summaries, requests.
  std::uint64_t flood_messages = 0;
  std::uint64_t flood_bytes = 0;
  std::uint64_t update_messages = 0;
  std::uint64_t update_bytes = 0;
  std::uint64_t route_changes = 0;
  std::uint64_t spf_runs = 0;
};

// An interval during which at least one forwarding loop existed.
struct LoopEpisode {
  Micros start = 0;
  Micros end = -1;  // -1 while still open
  std::size_t peak_pairs = 0;

  Micros duration() const { return end < 0 ? -1 : end - start; }
};

class Simulator {
 public:
  struct Params {
    Router::Config router;
    std::uint64_t seed = 1;
    // Loop detection walks every source and destination pair after every route
    // installation. Worth it for the loop experiment, wasteful for a pure
    // convergence sweep on a large network.
    bool detect_loops = true;
    // Routers do not put a recomputed table into force at the same instant:
    // programming a forwarding table takes a time that depends on the hardware
    // and on how many entries changed. Each router is given an installation
    // delay drawn once from the seeded source, uniform over [0, spread). Zero
    // means every router installs the moment it finishes computing, which is
    // an idealisation in which micro-loops cannot occur at all.
    Micros install_delay_spread = 0;
  };

  Simulator(Topology topology, Params params);
  ~Simulator();
  Simulator(const Simulator&) = delete;
  Simulator& operator=(const Simulator&) = delete;

  void run_until(Micros end);
  void run_for(Micros duration) { run_until(now_ + duration); }

  // Fault injection, all scheduled at a virtual time. `one_way` fails only the
  // a to b direction, which is the case that keeps hellos flowing one way and
  // makes the adjacency fall back rather than time out.
  void fail_link(Micros at, NodeId a, NodeId b, bool one_way = false);
  void restore_link(Micros at, NodeId a, NodeId b);
  void fail_node(Micros at, NodeId node);
  void restore_node(Micros at, NodeId node);

  Micros now() const { return now_; }
  const Topology& topology() const { return topology_; }
  const Router& router(NodeId id) const;
  const std::vector<LogEvent>& log() const { return log_; }
  const SimMetrics& metrics() const { return metrics_; }
  const std::vector<LoopEpisode>& loop_episodes() const { return loops_; }

  // The topology as it actually is at this instant: up nodes, and links that
  // are up in both directions.
  Graph truth() const;
  bool converged() const { return converged_; }
  // Virtual time at which the network last became fully converged.
  Micros last_converged_at() const { return converged_at_; }

  // Clears counters and loop history, leaving routing state alone. Called
  // after warm up so that a measurement counts the reconvergence and not the
  // initial start up flood.
  void mark_baseline();
  Micros baseline_time() const { return baseline_; }

  // Order independent digest of the whole event log. Two runs with the same
  // inputs must produce the same digest; that is the reproducibility check.
  std::uint64_t log_digest() const;

  std::string timeline_text() const;

 private:
  friend class NodeEnv;

  struct DirectedLink {
    bool up = true;
    Micros delay = 0;
    std::uint64_t bandwidth_bps = 1;
    Micros busy_until = 0;  // serialisation: one packet at a time on the wire
  };

  void schedule_at(Micros when, std::function<void()> fn);
  void transmit(NodeId from, NodeId to, const Bytes& datagram);
  void deliver(NodeId from, NodeId to, const Bytes& datagram);
  void record(const LogEvent& event);
  std::uint32_t next_random(std::uint32_t bound);
  bool node_up(NodeId id) const;
  bool link_up(NodeId from, NodeId to) const;
  void evaluate();
  void set_link(NodeId from, NodeId to, bool up);

  Topology topology_;
  Params params_;
  Micros now_ = 0;
  std::uint64_t sequence_ = 0;

  // Ordered by (time, insertion sequence). The sequence number is what makes
  // simultaneous events resolve in one fixed order instead of an arbitrary one.
  std::map<std::pair<Micros, std::uint64_t>, std::function<void()>> queue_;

  std::map<NodeId, std::unique_ptr<Router>> routers_;
  std::vector<std::unique_ptr<NodeEnv>> envs_;
  std::map<std::pair<NodeId, NodeId>, DirectedLink> links_;
  std::set<NodeId> failed_nodes_;

  std::mt19937_64 rng_;
  std::vector<LogEvent> log_;
  SimMetrics metrics_;
  std::vector<LoopEpisode> loops_;
  bool converged_ = false;
  Micros converged_at_ = -1;
  Micros baseline_ = 0;
};

}  // namespace waypoint
