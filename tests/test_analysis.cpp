#include "check.hpp"
#include "waypoint/analysis.hpp"
#include "waypoint/topology.hpp"

using namespace waypoint;

namespace {

Route to(NodeId next_hop, Cost cost) {
  Route r;
  r.cost = cost;
  r.next_hops = {next_hop};
  return r;
}

RibView view(const std::map<NodeId, RoutingTable>& tables) {
  RibView v;
  for (const auto& [id, table] : tables) v[id] = &table;
  return v;
}

}  // namespace

WP_TEST(analysis, consistent_tables_have_no_loop) {
  //  1 -> 2 -> 3, everyone agrees.
  std::map<NodeId, RoutingTable> tables;
  tables[1] = {{2, to(2, 1)}, {3, to(2, 2)}};
  tables[2] = {{1, to(1, 1)}, {3, to(3, 1)}};
  tables[3] = {{1, to(2, 2)}, {2, to(2, 1)}};
  CHECK_TRUE(find_forwarding_loops(view(tables)).empty());
}

WP_TEST(analysis, disagreement_between_two_nodes_is_a_loop) {
  // Node 1 still believes the path to 3 runs through 2; node 2 has already
  // recomputed and sends traffic for 3 back to 1. That is a micro-loop.
  std::map<NodeId, RoutingTable> tables;
  tables[1] = {{2, to(2, 1)}, {3, to(2, 2)}};
  tables[2] = {{1, to(1, 1)}, {3, to(1, 5)}};
  tables[3] = {};
  const auto loops = find_forwarding_loops(view(tables));
  CHECK_EQ(loops.size(), std::size_t{2});
  CHECK_TRUE(loops[0] == (std::pair<NodeId, NodeId>{1, 3}));
  CHECK_TRUE(loops[1] == (std::pair<NodeId, NodeId>{2, 3}));
}

WP_TEST(analysis, a_missing_route_is_a_black_hole_and_not_a_loop) {
  std::map<NodeId, RoutingTable> tables;
  tables[1] = {{2, to(2, 1)}, {3, to(2, 2)}};
  tables[2] = {{1, to(1, 1)}};  // node 2 has no route to 3 at all
  tables[3] = {};
  CHECK_TRUE(find_forwarding_loops(view(tables)).empty());

  const Topology t = topo::ring(3);
  const auto holes = find_black_holes(view(tables), t.graph(), {});
  CHECK_FALSE(holes.empty());
}

WP_TEST(analysis, tables_computed_from_the_truth_match_it) {
  const Topology ring = topo::ring(5);
  const Graph truth = ring.graph();
  std::map<NodeId, RoutingTable> tables;
  for (const NodeId n : ring.nodes) tables[n] = shortest_paths(truth, n);
  CHECK_TRUE(ribs_match_truth(view(tables), truth, {}));

  // Break one entry and the comparison must notice.
  tables[1][3].cost += 1;
  CHECK_FALSE(ribs_match_truth(view(tables), truth, {}));
  // Unless that node is excluded, which is how a failed node is handled.
  CHECK_TRUE(ribs_match_truth(view(tables), truth, {1}));
}

WP_TEST(analysis, a_stale_extra_route_also_counts_as_a_mismatch) {
  const Topology ring = topo::ring(4);
  const Graph truth = ring.graph();
  std::map<NodeId, RoutingTable> tables;
  for (const NodeId n : ring.nodes) tables[n] = shortest_paths(truth, n);
  tables[2][99] = to(3, 7);  // a destination that does not exist any more
  CHECK_FALSE(ribs_match_truth(view(tables), truth, {}));
}
