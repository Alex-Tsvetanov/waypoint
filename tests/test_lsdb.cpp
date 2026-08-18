#include "check.hpp"
#include "waypoint/lsdb.hpp"

using namespace waypoint;

namespace {
Lsa make_lsa(NodeId origin, std::uint32_t seq, AdjacencyList links = {}) {
  Lsa l;
  l.origin = origin;
  l.seq = seq;
  l.links = std::move(links);
  return l;
}
}  // namespace

WP_TEST(lsdb, newer_sequence_replaces_older) {
  Lsdb db;
  CHECK_TRUE(db.install(make_lsa(1, 5, {{2, 10}}), 0) == Lsdb::Result::Installed);
  CHECK_TRUE(db.install(make_lsa(1, 6, {{2, 20}}), kSec) == Lsdb::Result::Installed);
  CHECK_EQ(db.size(), std::size_t{1});
  const Lsa* held = db.find(1);
  CHECK_TRUE(held != nullptr);
  CHECK_EQ(held->seq, 6u);
  CHECK_EQ(held->links.at(0).cost, 20u);
}

WP_TEST(lsdb, duplicate_and_older_are_refused) {
  Lsdb db;
  db.install(make_lsa(1, 5), 0);
  // A duplicate must be distinguishable from an older copy: one is answered
  // with an acknowledgement, the other with the newer copy we hold.
  CHECK_TRUE(db.install(make_lsa(1, 5), kSec) == Lsdb::Result::Duplicate);
  CHECK_TRUE(db.install(make_lsa(1, 4), kSec) == Lsdb::Result::Older);
  CHECK_EQ(db.find(1)->seq, 5u);
}

WP_TEST(lsdb, sequence_wrap_is_accepted_as_newer) {
  Lsdb db;
  db.install(make_lsa(1, 0xFFFFFFFEu), 0);
  CHECK_TRUE(db.install(make_lsa(1, 0xFFFFFFFFu), 0) == Lsdb::Result::Installed);
  CHECK_TRUE(db.install(make_lsa(1, 0), 0) == Lsdb::Result::Installed);
  CHECK_EQ(db.find(1)->seq, 0u);
}

WP_TEST(lsdb, age_grows_with_held_time) {
  Lsdb db;
  Lsa l = make_lsa(1, 1);
  l.age_sec = 10;
  db.install(l, 0);
  CHECK_EQ(db.age_of(1, 0), std::uint16_t{10});
  CHECK_EQ(db.age_of(1, 30 * kSec), std::uint16_t{40});
  const auto snap = db.snapshot(30 * kSec);
  CHECK_EQ(snap.size(), std::size_t{1});
  CHECK_EQ(snap[0].age_sec, std::uint16_t{40});
}

WP_TEST(lsdb, maximum_age_entry_is_purged) {
  Lsdb db;
  db.install(make_lsa(1, 1), 0);
  db.install(make_lsa(2, 1), 0);
  CHECK_EQ(db.purge_expired(10 * kSec), std::size_t{0});
  CHECK_EQ(db.purge_expired(static_cast<Micros>(kMaxAgeSec) * kSec), std::size_t{2});
  CHECK_TRUE(db.empty());
}

WP_TEST(lsdb, maximum_age_advertisement_withdraws_the_origin) {
  Lsdb db;
  db.install(make_lsa(1, 1, {{2, 5}}), 0);
  Lsa flush = make_lsa(1, 2, {{2, 5}});
  flush.age_sec = kMaxAgeSec;
  CHECK_TRUE(db.install(flush, kSec) == Lsdb::Result::Expired);
  CHECK_TRUE(db.find(1) == nullptr);

  // A maximum age advertisement for something we never held is not installed
  // and, crucially, is not reflooded: Ignored and Expired are different orders.
  Lsdb fresh;
  CHECK_TRUE(fresh.install(flush, 0) == Lsdb::Result::Ignored);
  CHECK_TRUE(fresh.empty());
}

WP_TEST(lsdb, graph_keeps_one_sided_edges_visible) {
  Lsdb db;
  db.install(make_lsa(1, 1, {{2, 10}}), 0);
  db.install(make_lsa(2, 1, {}), 0);  // node 2 does not advertise the link back

  const Graph directed = db.graph();
  CHECK_TRUE(directed.has_edge(1, 2));
  CHECK_FALSE(directed.has_edge(2, 1));

  // The bidirectional view, which is what the route computation runs on,
  // drops the one sided edge entirely.
  const Graph usable = directed.bidirectional();
  CHECK_FALSE(usable.has_edge(1, 2));
  CHECK_EQ(usable.edge_count(), std::size_t{0});
}

WP_TEST(lsdb, summary_is_ordered_and_complete) {
  Lsdb db;
  db.install(make_lsa(3, 7), 0);
  db.install(make_lsa(1, 4), 0);
  db.install(make_lsa(2, 9), 0);
  const auto summary = db.summary();
  CHECK_EQ(summary.size(), std::size_t{3});
  CHECK_EQ(summary[0].origin, 1u);
  CHECK_EQ(summary[1].origin, 2u);
  CHECK_EQ(summary[2].origin, 3u);
  CHECK_EQ(summary[2].seq, 7u);
}
