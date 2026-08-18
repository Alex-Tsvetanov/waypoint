#include "waypoint/router.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace waypoint {

Router::Router(Config config, Env& env) : config_(config), env_(env) {}

void Router::add_link(NodeId peer, Cost cost) {
  links_[peer] = cost;
  Neighbor& n = neighbors_[peer];
  n.cost = cost;
}

void Router::start() {
  if (started_) return;
  started_ = true;

  // The first hello is offset by a random point within one whole interval, as
  // RFC 2328 requires, so that routers started together do not transmit on the
  // same tick for ever after. The offset must span the full interval and not
  // some fixed fraction of it: the phase between the last hello and a failure
  // is the only thing that varies between repetitions of an experiment, so a
  // capped offset would silently narrow the sample. The randomness comes from
  // Env, so it is seeded and reproducible.
  const Micros offset =
      config_.hello_interval > 0
          ? static_cast<Micros>(env_.random(static_cast<std::uint32_t>(
                std::min<Micros>(config_.hello_interval, 2000000000))))
          : 0;
  env_.schedule(offset, [this] { send_hello(); });

  if (config_.lsa_refresh_interval > 0) {
    env_.schedule(config_.lsa_refresh_interval, [this] {
      // Periodic refresh keeps our advertisement from ageing out of everyone
      // else's database. It reschedules itself for as long as the router runs.
      originate_lsa();
      if (config_.lsa_refresh_interval > 0) {
        env_.schedule(config_.lsa_refresh_interval, [this] { originate_lsa(); });
      }
    });
  }

  originate_lsa();
}

void Router::emit(EventKind kind, NodeId peer, std::uint32_t a, std::uint32_t b) {
  LogEvent e;
  e.time = env_.now();
  e.node = config_.id;
  e.kind = kind;
  e.peer = peer;
  e.a = a;
  e.b = b;
  env_.log(e);
}

NeighborState Router::state_of(NodeId peer) const {
  const auto it = neighbors_.find(peer);
  return it == neighbors_.end() ? NeighborState::Down : it->second.state;
}

// --- hello -----------------------------------------------------------------

void Router::send_hello() {
  Hello h;
  h.hello_interval_ms = static_cast<std::uint32_t>(config_.hello_interval / kMs);
  h.dead_interval_ms = static_cast<std::uint32_t>(config_.dead_interval / kMs);
  for (const auto& [peer, n] : neighbors_) {
    // Listing a neighbour is how we tell it we can hear it. That is the whole
    // of the two way check.
    if (n.state >= NeighborState::Init) h.seen.push_back(peer);
  }
  const Bytes wire = encode_hello(config_.id, h);
  for (const auto& [peer, cost] : links_) {
    (void)cost;
    env_.send(config_.id, peer, wire);
  }
  if (config_.hello_interval > 0) {
    env_.schedule(config_.hello_interval, [this] { send_hello(); });
  }
}

void Router::arm_dead_timer(NodeId peer) {
  Neighbor& n = neighbors_[peer];
  const std::uint64_t gen = ++n.dead_gen;
  env_.schedule(config_.dead_interval, [this, peer, gen] {
    const auto it = neighbors_.find(peer);
    // A superseded timer is recognised by its stamp and returns. This is why
    // Env needs no cancel operation.
    if (it == neighbors_.end() || it->second.dead_gen != gen) return;
    apply(peer, NeighborEvent::InactivityTimer);
  });
}

// --- adjacency -------------------------------------------------------------

void Router::apply(NodeId peer, NeighborEvent event) {
  const auto it = neighbors_.find(peer);
  if (it == neighbors_.end()) return;
  Neighbor& n = it->second;

  const Transition t = neighbor_transition(n.state, event);
  const NeighborState previous = n.state;
  if (t.next != previous) {
    n.state = t.next;
    emit(EventKind::NeighborState, peer, static_cast<std::uint32_t>(previous),
         static_cast<std::uint32_t>(t.next));
  }

  switch (t.action) {
    case NeighborAction::None:
      break;

    case NeighborAction::SendDbDescription:
      n.pending.clear();
      // Unsolicited. Whatever state the far end is in, it answers with its
      // current summary, so what comes back describes the database as it is
      // now and not as it was when the neighbour first noticed us.
      send_db_description(peer, /*reply=*/false);
      arm_dd_retransmit(peer);
      apply(peer, NeighborEvent::NegotiationDone);
      break;

    case NeighborAction::SendRequests: {
      n.request_attempts = 0;
      LsRequest req;
      req.origins.assign(n.pending.begin(), n.pending.end());
      if (!req.origins.empty()) {
        env_.send(config_.id, peer, encode_ls_request(config_.id, req));
        arm_retransmit();
      } else {
        apply(peer, NeighborEvent::LoadingDone);
      }
      break;
    }

    case NeighborAction::AdjacencyUp:
      ++n.dd_gen;  // stop retransmitting the summary
      originate_lsa();
      break;

    case NeighborAction::ClearAdjacency:
      ++n.dd_gen;
      n.rxmt.clear();
      n.pending.clear();
      n.request_attempts = 0;
      originate_lsa();
      schedule_spf();
      break;
  }
}

void Router::send_db_description(NodeId peer, bool reply) {
  DbDescription d;
  d.reply = reply;
  d.summary = lsdb_.summary();
  env_.send(config_.id, peer, encode_db_description(config_.id, d));
}

void Router::arm_dd_retransmit(NodeId peer) {
  const auto it = neighbors_.find(peer);
  if (it == neighbors_.end()) return;
  const std::uint64_t gen = it->second.dd_gen;
  env_.schedule(config_.rxmt_interval, [this, peer, gen] {
    const auto n = neighbors_.find(peer);
    if (n == neighbors_.end() || n->second.dd_gen != gen) return;
    if (n->second.state != NeighborState::ExStart &&
        n->second.state != NeighborState::Exchange) {
      return;
    }
    // The far end may still have been in Init when our summary arrived and
    // dropped it. Repeating it is cheaper than a handshake for the case.
    send_db_description(peer, /*reply=*/false);
    arm_dd_retransmit(peer);
  });
}

// --- flooding --------------------------------------------------------------

void Router::flood(const Lsa& lsa, NodeId exclude) {
  LsUpdate u;
  u.lsas = {lsa};
  const Bytes wire = encode_ls_update(config_.id, u);
  std::uint32_t sent = 0;
  for (auto& [peer, n] : neighbors_) {
    // RFC 2328 floods to every adjacency in Exchange or above, not only to the
    // finished ones. Waiting for Full would leave a router that is still
    // loading permanently short of anything learned in the meantime.
    if (n.state < NeighborState::Exchange) continue;
    if (peer == exclude) continue;
    n.rxmt[lsa.origin] = lsa;
    env_.send(config_.id, peer, wire);
    ++sent;
  }
  if (sent > 0) {
    arm_retransmit();
    emit(EventKind::LsaReflooded, lsa.origin, sent, lsa.seq);
  }
}

void Router::arm_retransmit() {
  if (rxmt_armed_) return;
  rxmt_armed_ = true;
  env_.schedule(config_.rxmt_interval, [this] {
    rxmt_armed_ = false;
    bool outstanding = false;
    std::vector<NodeId> gave_up;
    for (auto& [peer, n] : neighbors_) {
      if (n.state >= NeighborState::Exchange && !n.rxmt.empty()) {
        LsUpdate u;
        for (const auto& [origin, lsa] : n.rxmt) {
          (void)origin;
          u.lsas.push_back(lsa);
        }
        env_.send(config_.id, peer, encode_ls_update(config_.id, u));
        outstanding = true;
      }
      if (n.state == NeighborState::Loading && !n.pending.empty()) {
        if (++n.request_attempts > kMaxRequestAttempts) {
          n.pending.clear();
          gave_up.push_back(peer);
        } else {
          LsRequest req;
          req.origins.assign(n.pending.begin(), n.pending.end());
          env_.send(config_.id, peer, encode_ls_request(config_.id, req));
          outstanding = true;
        }
      }
    }
    // Applied outside the loop: bringing an adjacency up originates a new
    // advertisement, which walks the same container.
    for (const NodeId peer : gave_up) apply(peer, NeighborEvent::LoadingDone);
    if (outstanding) arm_retransmit();
  });
}

void Router::originate_lsa() {
  const Micros now = env_.now();
  if (config_.min_lsa_interval > 0 && last_origination_ >= 0 &&
      now - last_origination_ < config_.min_lsa_interval) {
    // Pacing. The change is not lost, it is deferred to the earliest moment
    // the pacing rule allows, and a single deferred origination covers any
    // number of changes that arrive in the meantime.
    if (!origination_pending_) {
      origination_pending_ = true;
      const Micros delay = last_origination_ + config_.min_lsa_interval - now;
      env_.schedule(delay, [this] {
        origination_pending_ = false;
        originate_lsa();
      });
    }
    return;
  }

  Lsa lsa;
  lsa.origin = config_.id;
  lsa.seq = ++own_seq_;
  lsa.age_sec = 0;
  for (const auto& [peer, n] : neighbors_) {
    if (n.state == NeighborState::Full) lsa.links.push_back(Adjacency{peer, n.cost});
  }
  lsdb_.install(lsa, now);
  last_origination_ = now;
  ++originations_;
  emit(EventKind::LsaOriginated, config_.id, lsa.seq,
       static_cast<std::uint32_t>(lsa.links.size()));
  flood(lsa, kNoNode);
  schedule_spf();
}

// --- receive ---------------------------------------------------------------

void Router::receive(ByteView datagram) {
  const auto header = decode_header(datagram);
  if (!header) {
    ++malformed_;
    return;
  }
  // Only configured interfaces are listened to. A datagram from anyone else is
  // discarded before it can touch protocol state.
  if (links_.find(header->sender) == links_.end()) {
    ++malformed_;
    return;
  }
  const NodeId from = header->sender;

  switch (header->type) {
    case PacketType::Hello: handle_hello(from, datagram); break;
    case PacketType::DbDescription: handle_db_description(from, datagram); break;
    case PacketType::LsRequest: handle_ls_request(from, datagram); break;
    case PacketType::LsUpdate: handle_ls_update(from, datagram); break;
    case PacketType::LsAck: handle_ls_ack(from, datagram); break;
  }
}

void Router::handle_hello(NodeId from, ByteView datagram) {
  const auto hello = decode_hello(datagram);
  if (!hello) {
    ++malformed_;
    return;
  }
  arm_dead_timer(from);
  apply(from, NeighborEvent::HelloReceived);

  const bool listed = std::find(hello->seen.begin(), hello->seen.end(),
                                config_.id) != hello->seen.end();
  apply(from, listed ? NeighborEvent::TwoWayReceived
                     : NeighborEvent::OneWayReceived);
}

void Router::handle_db_description(NodeId from, ByteView datagram) {
  const auto dd = decode_db_description(datagram);
  if (!dd) {
    ++malformed_;
    return;
  }
  const auto it = neighbors_.find(from);
  if (it == neighbors_.end()) return;
  // Below Exchange there is nothing to synchronise against yet. The neighbour
  // repeats the summary on its retransmission timer, and once we reach
  // Exchange our own unsolicited summary draws a fresh answer out of it.
  if (it->second.state < NeighborState::Exchange) return;

  process_summary(from, dd->summary);
  if (!dd->reply) send_db_description(from, /*reply=*/true);
}

void Router::process_summary(NodeId peer, const std::vector<LsaHandle>& summary) {
  const auto it = neighbors_.find(peer);
  if (it == neighbors_.end()) return;
  Neighbor& n = it->second;

  n.pending.clear();
  for (const LsaHandle& s : summary) {
    const Lsa* held = lsdb_.find(s.origin);
    if (held == nullptr || seq_newer(s.seq, held->seq)) n.pending.insert(s.origin);
  }
  if (n.state == NeighborState::Exchange) {
    apply(peer, NeighborEvent::ExchangeDone);
  } else if (n.state == NeighborState::Loading && n.pending.empty()) {
    apply(peer, NeighborEvent::LoadingDone);
  }
}

void Router::handle_ls_request(NodeId from, ByteView datagram) {
  const auto req = decode_ls_request(datagram);
  if (!req) {
    ++malformed_;
    return;
  }
  LsUpdate u;
  for (const NodeId origin : req->origins) {
    if (auto lsa = lsdb_.get(origin, env_.now())) u.lsas.push_back(std::move(*lsa));
  }
  if (!u.lsas.empty()) {
    env_.send(config_.id, from, encode_ls_update(config_.id, u));
  }
}

void Router::handle_ls_update(NodeId from, ByteView datagram) {
  const auto update = decode_ls_update(datagram);
  if (!update) {
    ++malformed_;
    return;
  }
  const auto it = neighbors_.find(from);
  if (it == neighbors_.end()) return;

  LsAck ack;
  bool database_changed = false;

  for (const Lsa& lsa : update->lsas) {
    ack.acked.push_back(LsaHandle{lsa.origin, lsa.seq});

    if (lsa.origin == config_.id) {
      // Our own advertisement came back with a sequence number ahead of ours,
      // which happens after a restart. Jump past it and re-originate, or the
      // network keeps believing a version of us that no longer exists.
      if (seq_newer(lsa.seq, own_seq_)) {
        own_seq_ = lsa.seq;
        originate_lsa();
      }
      continue;
    }

    const Lsdb::Result result = lsdb_.install(lsa, env_.now());
    // Whatever the database made of it, the request for this origin has been
    // answered. Leaving it outstanding when the copy that arrived turned out
    // to be older than one already learned elsewhere would hold the adjacency
    // in Loading for the rest of the run.
    it->second.pending.erase(lsa.origin);
    switch (result) {
      case Lsdb::Result::Installed:
        database_changed = true;
        emit(EventKind::LsaInstalled, lsa.origin, lsa.seq,
             static_cast<std::uint32_t>(lsa.links.size()));
        flood(lsa, from);
        break;
      case Lsdb::Result::Expired:
        database_changed = true;
        flood(lsa, from);
        break;
      case Lsdb::Result::Duplicate:
        // Already held. Acknowledge and stop: this is the duplicate
        // suppression that keeps flooding from circulating for ever.
        it->second.rxmt.erase(lsa.origin);
        break;
      case Lsdb::Result::Older: {
        // We hold something newer. Send it straight back rather than dropping
        // silently, so the neighbour converges without waiting for a refresh.
        if (auto newer = lsdb_.get(lsa.origin, env_.now())) {
          LsUpdate reply;
          reply.lsas.push_back(std::move(*newer));
          env_.send(config_.id, from, encode_ls_update(config_.id, reply));
        }
        break;
      }
      case Lsdb::Result::Ignored:
        break;
    }
  }

  if (!ack.acked.empty()) {
    env_.send(config_.id, from, encode_ls_ack(config_.id, ack));
  }
  if (database_changed) schedule_spf();
  if (it->second.state == NeighborState::Loading && it->second.pending.empty()) {
    apply(from, NeighborEvent::LoadingDone);
  }
}

void Router::handle_ls_ack(NodeId from, ByteView datagram) {
  const auto ack = decode_ls_ack(datagram);
  if (!ack) {
    ++malformed_;
    return;
  }
  const auto it = neighbors_.find(from);
  if (it == neighbors_.end()) return;
  for (const LsaHandle& s : ack->acked) {
    const auto held = it->second.rxmt.find(s.origin);
    if (held != it->second.rxmt.end() && held->second.seq == s.seq) {
      it->second.rxmt.erase(held);
    }
  }
}

// --- route computation -----------------------------------------------------

void Router::schedule_spf() {
  if (spf_pending_) return;
  spf_pending_ = true;
  env_.schedule(config_.spf_delay, [this] {
    spf_pending_ = false;
    run_spf();
  });
}

void Router::run_spf() {
  const Graph usable = lsdb_.graph().bidirectional();
  RoutingTable next = shortest_paths(usable, config_.id);
  ++spf_runs_;
  emit(EventKind::SpfComputed, config_.id, spf_runs_,
       static_cast<std::uint32_t>(next.size()));

  if (config_.install_delay > 0) {
    env_.schedule(config_.install_delay,
                  [this, next = std::move(next)]() mutable {
                    install(std::move(next));
                  });
  } else {
    install(std::move(next));
  }
}

void Router::install(RoutingTable next) {
  std::uint32_t changed = 0;
  for (const auto& [dest, route] : next) {
    const auto old = rib_.find(dest);
    if (old == rib_.end() || !(old->second == route)) ++changed;
  }
  for (const auto& [dest, route] : rib_) {
    (void)route;
    if (next.find(dest) == next.end()) ++changed;
  }

  rib_ = std::move(next);
  route_changes_ += changed;
  // Logged after the table is in force, so an observer that reacts to this
  // event reads the routing table the router is actually using.
  emit(EventKind::RouteInstalled, config_.id, changed,
       static_cast<std::uint32_t>(rib_.size()));
}

}  // namespace waypoint
