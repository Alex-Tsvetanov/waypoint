#include "check.hpp"
#include "waypoint/sim.hpp"

using namespace waypoint;

namespace {

Simulator::Params fast_params(std::uint64_t seed = 1) {
  Simulator::Params p;
  p.router.hello_interval = 200 * kMs;
  p.router.dead_interval = 800 * kMs;
  p.router.rxmt_interval = 200 * kMs;
  p.router.spf_delay = 20 * kMs;
  p.seed = seed;
  return p;
}

// Runs a topology to a converged state and returns the simulator, so the
// individual cases start from a network that already agrees with itself.
std::unique_ptr<Simulator> converged_ring(std::size_t n, std::uint64_t seed = 1) {
  auto sim = std::make_unique<Simulator>(topo::ring(n), fast_params(seed));
  sim->run_until(10 * kSec);
  return sim;
}

}  // namespace

WP_TEST(sim, a_ring_converges_from_cold) {
  const auto sim = converged_ring(6);
  CHECK_TRUE(sim->converged());
  for (const NodeId n : sim->topology().nodes) {
    CHECK_EQ(sim->router(n).rib().size(), std::size_t{5});
    for (const auto& [peer, cost] : sim->router(n).links()) {
      (void)cost;
      CHECK_TRUE(sim->router(n).state_of(peer) == NeighborState::Full);
    }
    CHECK_FALSE(sim->router(n).malformed_seen());
  }
}

WP_TEST(sim, the_same_seed_reproduces_the_run_byte_for_byte) {
  const auto a = converged_ring(8, 12345);
  const auto b = converged_ring(8, 12345);
  CHECK_EQ(a->log().size(), b->log().size());
  CHECK_EQ(a->log_digest(), b->log_digest());
  CHECK_EQ(a->metrics().messages_sent, b->metrics().messages_sent);
  CHECK_EQ(a->metrics().bytes_sent, b->metrics().bytes_sent);
}

WP_TEST(sim, a_different_seed_still_converges) {
  // Reproducibility must not be an accident of one seed.
  for (const std::uint64_t seed : {2ull, 99ull, 7777ull}) {
    const auto sim = converged_ring(7, seed);
    CHECK_TRUE(sim->converged());
  }
}

WP_TEST(sim, equal_cost_paths_appear_in_a_ring_of_even_size) {
  const auto sim = converged_ring(6);
  // From node 1 the node opposite is three hops either way round.
  const Route& opposite = sim->router(1).rib().at(4);
  CHECK_EQ(opposite.cost, 3u);
  CHECK_EQ(opposite.next_hops.size(), std::size_t{2});
}

WP_TEST(sim, a_link_failure_is_detected_and_routed_around) {
  auto sim = converged_ring(6);
  const NodeId old_hop = sim->router(1).rib().at(2).next_hops.front();
  CHECK_EQ(old_hop, 2u);

  sim->mark_baseline();
  const Micros failure = sim->now() + 1 * kSec;
  sim->fail_link(failure, 1, 2);
  sim->run_until(failure + 15 * kSec);

  CHECK_TRUE(sim->converged());
  CHECK_TRUE(sim->last_converged_at() > failure);
  // The ring is now a line, so node 1 reaches node 2 the long way round.
  CHECK_EQ(sim->router(1).rib().at(2).cost, 5u);
  CHECK_EQ(sim->router(1).rib().at(2).next_hops.front(), 6u);
  // Detection cannot be faster than the dead interval, and should not be much
  // slower than the dead interval plus the flooding of one advertisement.
  const Micros reconvergence = sim->last_converged_at() - failure;
  CHECK_TRUE(reconvergence >= 0);
  CHECK_TRUE(reconvergence < 10 * kSec);
}

WP_TEST(sim, a_restored_link_is_taken_back_into_use) {
  auto sim = converged_ring(6);
  const Micros failure = sim->now() + 1 * kSec;
  sim->fail_link(failure, 1, 2);
  sim->run_until(failure + 15 * kSec);
  CHECK_EQ(sim->router(1).rib().at(2).cost, 5u);

  sim->restore_link(sim->now() + 1 * kSec, 1, 2);
  sim->run_until(sim->now() + 20 * kSec);
  CHECK_TRUE(sim->converged());
  CHECK_EQ(sim->router(1).rib().at(2).cost, 1u);
}

WP_TEST(sim, a_one_way_failure_also_reconverges) {
  // Only packets from 1 to 2 are lost. Node 2 times out and stops listing
  // node 1, node 1 sees the one way hello and falls back. Neither end may keep
  // advertising the link.
  auto sim = std::make_unique<Simulator>(topo::ring(6), fast_params(4));
  sim->run_until(10 * kSec);
  CHECK_TRUE(sim->converged());

  const Micros failure = sim->now() + 1 * kSec;
  sim->fail_link(failure, 1, 2, /*one_way=*/true);
  sim->run_until(failure + 20 * kSec);

  CHECK_TRUE(sim->converged());
  CHECK_EQ(sim->router(1).rib().at(2).cost, 5u);
  CHECK_TRUE(sim->router(1).state_of(2) != NeighborState::Full);
  CHECK_TRUE(sim->router(2).state_of(1) != NeighborState::Full);
}

WP_TEST(sim, a_failed_node_disappears_from_every_table) {
  auto sim = std::make_unique<Simulator>(topo::grid_mesh(3, 3), fast_params(5));
  sim->run_until(12 * kSec);
  CHECK_TRUE(sim->converged());

  const Micros failure = sim->now() + 1 * kSec;
  sim->fail_node(failure, 5);  // the centre of the grid
  sim->run_until(failure + 20 * kSec);

  CHECK_TRUE(sim->converged());
  for (const NodeId n : sim->topology().nodes) {
    if (n == 5) continue;
    CHECK_TRUE(sim->router(n).rib().find(5) == sim->router(n).rib().end());
    CHECK_EQ(sim->router(n).rib().size(), std::size_t{7});
  }
}

WP_TEST(sim, flooding_overhead_is_counted_separately_from_hellos) {
  auto sim = converged_ring(6);
  sim->mark_baseline();
  const SimMetrics before = sim->metrics();
  CHECK_EQ(before.messages_sent, std::uint64_t{0});

  const Micros failure = sim->now() + 1 * kSec;
  sim->fail_link(failure, 1, 2);
  sim->run_until(failure + 10 * kSec);

  const SimMetrics after = sim->metrics();
  CHECK_TRUE(after.update_messages > 0);
  CHECK_TRUE(after.update_bytes > 0);
  CHECK_TRUE(after.hello_messages > 0);
  CHECK_EQ(after.messages_sent, after.hello_messages + after.flood_messages);
  CHECK_EQ(after.bytes_sent, after.hello_bytes + after.flood_bytes);
  // Packets offered to the failed link are dropped, not delivered.
  CHECK_TRUE(after.messages_dropped > 0);
}

WP_TEST(sim, flooding_terminates) {
  // Duplicate suppression is the only thing stopping a ring from circulating
  // an advertisement for ever. With the network quiet, the message count must
  // stop growing except for hellos.
  auto sim = converged_ring(8);
  sim->mark_baseline();
  sim->run_until(sim->now() + 5 * kSec);
  const std::uint64_t updates = sim->metrics().update_messages;
  sim->run_until(sim->now() + 5 * kSec);
  CHECK_EQ(sim->metrics().update_messages, updates);
}

WP_TEST(sim, a_fat_tree_converges_and_uses_equal_cost_paths) {
  auto sim = std::make_unique<Simulator>(topo::fat_tree(4), fast_params(3));
  sim->run_until(20 * kSec);
  CHECK_TRUE(sim->converged());
  // An edge switch reaching another pod has two aggregation switches to
  // choose from, so the multipath set must have more than one member.
  const RoutingTable& table = sim->router(7).rib();
  std::size_t multipath = 0;
  for (const auto& [dest, route] : table) {
    (void)dest;
    if (route.next_hops.size() > 1) ++multipath;
  }
  CHECK_TRUE(multipath > 0);
}

WP_TEST(sim, an_answered_request_never_leaves_an_adjacency_loading) {
  // Regression. A router asks a neighbour for an advertisement named in its
  // summary, but a newer copy of that same advertisement arrives from a third
  // router first. The answer then looks older than what is already held. If
  // that outcome does not clear the request, the adjacency waits in Loading
  // for ever and the link never enters anyone's topology. Seed 1001 on a four
  // by four mesh reproduced it every time.
  auto sim = std::make_unique<Simulator>(topo::grid_mesh(4, 4), fast_params(1001));
  sim->run_until(20 * kSec);
  CHECK_TRUE(sim->converged());
  for (const NodeId n : sim->topology().nodes) {
    for (const auto& [peer, cost] : sim->router(n).links()) {
      (void)cost;
      CHECK_TRUE(sim->router(n).state_of(peer) == NeighborState::Full);
    }
    CHECK_EQ(sim->router(n).rib().size(), std::size_t{15});
  }
}

WP_TEST(sim, a_summary_that_arrives_early_does_not_deadlock_the_exchange) {
  // Regression. One end reaches ExStart a hello interval before the other and
  // sends its summary into a router still in Init. If that summary is simply
  // dropped and never asked for again, both ends wait for each other. Forty
  // seeds, because the race depends on the phase of the hello timers.
  for (std::uint64_t seed = 1; seed <= 40; ++seed) {
    const auto sim = converged_ring(10, seed);
    CHECK_TRUE(sim->converged());
  }
}

WP_TEST(sim, a_ring_produces_a_transient_loop_and_a_mesh_does_not) {
  // With every router installing its new table at the same instant a loop is
  // impossible by construction, so the routers are given a spread of
  // installation delays. Then a ring loops, because the far side of the break
  // has to reverse direction, and a grid mesh does not, because its
  // alternative path leaves every surviving edge pointing the way it already
  // pointed.
  auto looping = [](Topology t, NodeId a, NodeId b) {
    Simulator::Params p = fast_params(11);
    p.install_delay_spread = 400 * kMs;
    p.detect_loops = true;
    Simulator sim(std::move(t), p);
    sim.run_until(15 * kSec);
    sim.mark_baseline();
    const Micros failure = sim.now() + 1 * kSec;
    sim.fail_link(failure, a, b);
    sim.run_until(failure + 20 * kSec);
    return std::make_pair(sim.converged(), sim.loop_episodes().size());
  };

  const auto ring = looping(topo::ring(12), 6, 7);
  CHECK_TRUE(ring.first);
  CHECK_TRUE(ring.second > 0);

  const auto mesh = looping(topo::grid_mesh(4, 4), 6, 7);
  CHECK_TRUE(mesh.first);
  CHECK_EQ(mesh.second, std::size_t{0});
}

WP_TEST(sim, a_loop_episode_is_closed_when_it_ends) {
  Simulator::Params p = fast_params(11);
  p.install_delay_spread = 400 * kMs;
  Simulator sim(topo::ring(12), p);
  sim.run_until(15 * kSec);
  sim.mark_baseline();
  const Micros failure = sim.now() + 1 * kSec;
  sim.fail_link(failure, 6, 7);
  sim.run_until(failure + 20 * kSec);
  CHECK_TRUE(!sim.loop_episodes().empty());
  for (const LoopEpisode& l : sim.loop_episodes()) {
    CHECK_TRUE(l.end > l.start);        // no episode left open at the end
    CHECK_TRUE(l.duration() > 0);
    CHECK_TRUE(l.start >= failure);     // and none predates the failure
    CHECK_TRUE(l.peak_pairs > 0);
  }
}

WP_TEST(sim, cold_start_converges_on_every_ring_over_forty_seeds) {
  // The initial convergence check behind the claim in section VI: seven rings
  // at the default timers, forty seeds each, no run allowed to fail and the
  // slowest start up reported exactly. The value is an equality and not a
  // bound on purpose. A deterministic simulator that changes this number has
  // changed its behaviour, and that is what the report is asserting.
  Micros slowest = -1;
  for (const std::size_t n : {4u, 6u, 8u, 12u, 16u, 24u, 32u}) {
    for (std::uint64_t seed = 1000; seed <= 1039; ++seed) {
      Simulator::Params p;  // defaults: hello 1 s, dead 4 s, spf delay 100 ms
      p.seed = seed;
      p.detect_loops = false;
      Simulator sim(topo::ring(n), p);
      sim.run_until(40 * kSec);
      CHECK_TRUE(sim.converged());
      if (sim.last_converged_at() > slowest) slowest = sim.last_converged_at();
    }
  }
  CHECK_EQ(slowest, Micros{2098035});
}
