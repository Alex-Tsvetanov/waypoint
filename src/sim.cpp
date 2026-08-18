#include "waypoint/sim.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace waypoint {

// The Env every simulated router is given. It holds a node id and a pointer
// back to the simulator, and nothing else: all the behaviour lives in the
// simulator, so there is only one place where virtual time advances.
class NodeEnv final : public Env {
 public:
  NodeEnv(Simulator& sim, NodeId id) : sim_(sim), id_(id) {}

  Micros now() const override { return sim_.now_; }

  void schedule(Micros delay, std::function<void()> fn) override {
    sim_.schedule_at(sim_.now_ + (delay < 0 ? 0 : delay), std::move(fn));
  }

  void send(NodeId self, NodeId peer, const Bytes& datagram) override {
    sim_.transmit(self, peer, datagram);
  }

  std::uint32_t random(std::uint32_t bound) override { return sim_.next_random(bound); }

  void log(const LogEvent& event) override { sim_.record(event); }

  NodeId id() const { return id_; }

 private:
  Simulator& sim_;
  NodeId id_;
};

Simulator::Simulator(Topology topology, Params params)
    : topology_(std::move(topology)), params_(params), rng_(params.seed) {
  for (const TopologyLink& l : topology_.links) {
    DirectedLink forward;
    forward.delay = l.delay;
    forward.bandwidth_bps = l.bandwidth_bps == 0 ? 1 : l.bandwidth_bps;
    links_[{l.a, l.b}] = forward;
    links_[{l.b, l.a}] = forward;
  }

  for (const NodeId id : topology_.nodes) {
    Router::Config config = params_.router;
    config.id = id;
    if (params_.install_delay_spread > 0) {
      config.install_delay +=
          static_cast<Micros>(next_random(static_cast<std::uint32_t>(
              params_.install_delay_spread)));
    }
    envs_.push_back(std::make_unique<NodeEnv>(*this, id));
    routers_[id] = std::make_unique<Router>(config, *envs_.back());
  }
  for (const TopologyLink& l : topology_.links) {
    routers_.at(l.a)->add_link(l.b, l.cost);
    routers_.at(l.b)->add_link(l.a, l.cost);
  }
  for (const NodeId id : topology_.nodes) routers_.at(id)->start();
}

Simulator::~Simulator() = default;

const Router& Simulator::router(NodeId id) const {
  const auto it = routers_.find(id);
  if (it == routers_.end()) throw std::out_of_range("no such router");
  return *it->second;
}

void Simulator::schedule_at(Micros when, std::function<void()> fn) {
  queue_.emplace(std::pair<Micros, std::uint64_t>{when, sequence_++}, std::move(fn));
}

std::uint32_t Simulator::next_random(std::uint32_t bound) {
  if (bound == 0) return 0;
  // Taken modulo on purpose: the bias is irrelevant for timer jitter and the
  // arithmetic is identical on every platform, which matters more here than
  // uniformity. std::uniform_int_distribution is not portable across
  // implementations.
  return static_cast<std::uint32_t>(rng_() % bound);
}

bool Simulator::node_up(NodeId id) const {
  return failed_nodes_.find(id) == failed_nodes_.end();
}

bool Simulator::link_up(NodeId from, NodeId to) const {
  const auto it = links_.find({from, to});
  return it != links_.end() && it->second.up;
}

void Simulator::transmit(NodeId from, NodeId to, const Bytes& datagram) {
  if (!node_up(from) || !node_up(to) || !link_up(from, to)) {
    ++metrics_.messages_dropped;
    return;
  }
  DirectedLink& link = links_.at({from, to});

  // Serialisation delay on the wire, then propagation. A second packet offered
  // while the first is still being clocked out queues behind it.
  const Micros transmission =
      static_cast<Micros>(static_cast<std::uint64_t>(datagram.size()) * 8 * 1000000 /
                          link.bandwidth_bps);
  const Micros start = std::max(now_, link.busy_until);
  link.busy_until = start + transmission;
  const Micros arrival = link.busy_until + link.delay;

  ++metrics_.messages_sent;
  metrics_.bytes_sent += datagram.size();
  if (const auto header = decode_header(datagram)) {
    if (header->type == PacketType::Hello) {
      ++metrics_.hello_messages;
      metrics_.hello_bytes += datagram.size();
    } else {
      ++metrics_.flood_messages;
      metrics_.flood_bytes += datagram.size();
      if (header->type == PacketType::LsUpdate) {
        ++metrics_.update_messages;
        metrics_.update_bytes += datagram.size();
      }
    }
  }

  schedule_at(arrival, [this, from, to, payload = datagram]() {
    deliver(from, to, payload);
  });
}

void Simulator::deliver(NodeId from, NodeId to, const Bytes& datagram) {
  // A packet already on the wire when the link or the far node fails is lost.
  if (!node_up(to) || !link_up(from, to)) {
    ++metrics_.messages_dropped;
    return;
  }
  routers_.at(to)->receive(datagram);
}

void Simulator::record(const LogEvent& event) {
  log_.push_back(event);
  if (event.kind == EventKind::SpfComputed) ++metrics_.spf_runs;
  if (event.kind == EventKind::RouteInstalled) {
    metrics_.route_changes += event.a;
    evaluate();
  }
}

Graph Simulator::truth() const {
  Graph g;
  for (const NodeId n : topology_.nodes) {
    if (node_up(n)) g.adj.try_emplace(n);
  }
  for (const TopologyLink& l : topology_.links) {
    if (!node_up(l.a) || !node_up(l.b)) continue;
    // A link usable in one direction only is not usable at all, because the
    // protocol refuses to compute over an edge that is not advertised by both
    // ends. The truth graph has to apply the same rule or the comparison would
    // never agree.
    if (!link_up(l.a, l.b) || !link_up(l.b, l.a)) continue;
    g.add_edge(l.a, l.b, l.cost);
    g.add_edge(l.b, l.a, l.cost);
  }
  return g;
}

void Simulator::evaluate() {
  RibView ribs;
  for (const auto& [id, router] : routers_) {
    if (!node_up(id)) continue;
    ribs[id] = &router->rib();
  }
  const Graph reference = truth();

  const bool was_converged = converged_;
  converged_ = ribs_match_truth(ribs, reference, failed_nodes_);
  if (converged_ && !was_converged) converged_at_ = now_;

  if (!params_.detect_loops) return;
  const auto pairs = find_forwarding_loops(ribs);
  const bool open = !loops_.empty() && loops_.back().end < 0;
  if (!pairs.empty()) {
    if (!open) {
      LoopEpisode episode;
      episode.start = now_;
      episode.peak_pairs = pairs.size();
      loops_.push_back(episode);
    } else {
      loops_.back().peak_pairs = std::max(loops_.back().peak_pairs, pairs.size());
    }
  } else if (open) {
    loops_.back().end = now_;
  }
}

void Simulator::set_link(NodeId from, NodeId to, bool up) {
  const auto it = links_.find({from, to});
  if (it == links_.end()) return;
  it->second.up = up;
}

void Simulator::fail_link(Micros at, NodeId a, NodeId b, bool one_way) {
  schedule_at(at, [this, a, b, one_way] {
    set_link(a, b, false);
    if (!one_way) set_link(b, a, false);
    LogEvent e;
    e.time = now_;
    e.node = a;
    e.peer = b;
    e.kind = EventKind::LinkFailure;
    e.a = one_way ? 1u : 0u;
    log_.push_back(e);
    evaluate();
  });
}

void Simulator::restore_link(Micros at, NodeId a, NodeId b) {
  schedule_at(at, [this, a, b] {
    set_link(a, b, true);
    set_link(b, a, true);
    // A restored link is idle: whatever was queued on it before the failure is
    // gone, not waiting.
    if (const auto it = links_.find({a, b}); it != links_.end()) it->second.busy_until = now_;
    if (const auto it = links_.find({b, a}); it != links_.end()) it->second.busy_until = now_;
    LogEvent e;
    e.time = now_;
    e.node = a;
    e.peer = b;
    e.kind = EventKind::LinkRestore;
    log_.push_back(e);
    evaluate();
  });
}

void Simulator::fail_node(Micros at, NodeId node) {
  schedule_at(at, [this, node] {
    // Total isolation. From every other router's point of view a halted node
    // and a node whose every link went down are the same observation, so one
    // mechanism covers both.
    failed_nodes_.insert(node);
    LogEvent e;
    e.time = now_;
    e.node = node;
    e.kind = EventKind::NodeFailure;
    log_.push_back(e);
    evaluate();
  });
}

void Simulator::restore_node(Micros at, NodeId node) {
  schedule_at(at, [this, node] {
    failed_nodes_.erase(node);
    LogEvent e;
    e.time = now_;
    e.node = node;
    e.kind = EventKind::NodeRestore;
    log_.push_back(e);
    evaluate();
  });
}

void Simulator::run_until(Micros end) {
  while (!queue_.empty()) {
    const auto it = queue_.begin();
    const Micros when = it->first.first;
    if (when > end) break;
    std::function<void()> fn = std::move(it->second);
    queue_.erase(it);
    now_ = when;
    fn();
  }
  now_ = std::max(now_, end);
}

void Simulator::mark_baseline() {
  metrics_ = SimMetrics{};
  loops_.clear();
  baseline_ = now_;
  converged_at_ = -1;
  evaluate();
}

std::uint64_t Simulator::log_digest() const {
  // FNV-1a over every field of every event. Written out rather than pulled
  // from a library so the digest is identical on any platform.
  std::uint64_t hash = 1469598103934665603ull;
  auto mix = [&hash](std::uint64_t value) {
    for (int i = 0; i < 8; ++i) {
      hash ^= (value >> (8 * i)) & 0xFF;
      hash *= 1099511628211ull;
    }
  };
  for (const LogEvent& e : log_) {
    mix(static_cast<std::uint64_t>(e.time));
    mix(e.node);
    mix(static_cast<std::uint64_t>(e.kind));
    mix(e.peer);
    mix(e.a);
    mix(e.b);
  }
  return hash;
}

std::string Simulator::timeline_text() const {
  std::ostringstream out;
  for (const LogEvent& e : log_) {
    out << e.time << " us  node " << e.node << "  " << to_string(e.kind);
    if (e.peer != kNoNode && e.peer != e.node) out << " peer=" << e.peer;
    out << " a=" << e.a << " b=" << e.b << "\n";
  }
  return out.str();
}

}  // namespace waypoint
