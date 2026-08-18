#include "check.hpp"
#include "waypoint/spf.hpp"

using namespace waypoint;

namespace {
// Builds an undirected graph by adding both directions of every link.
Graph undirected(std::initializer_list<std::tuple<NodeId, NodeId, Cost>> links) {
  Graph g;
  for (const auto& [a, b, c] : links) {
    g.add_edge(a, b, c);
    g.add_edge(b, a, c);
  }
  return g;
}
}  // namespace

WP_TEST(spf, single_shortest_path_chain) {
  //  1 --1-- 2 --1-- 3 --1-- 4
  const Graph g = undirected({{1, 2, 1}, {2, 3, 1}, {3, 4, 1}});
  const RoutingTable r = shortest_paths(g, 1);

  CHECK_EQ(r.size(), std::size_t{3});
  CHECK_EQ(r.at(4).cost, 3u);
  CHECK_EQ(r.at(4).next_hops.size(), std::size_t{1});
  CHECK_EQ(r.at(4).next_hops[0], 2u);
  CHECK_EQ(r.at(2).next_hops[0], 2u);
  CHECK_TRUE(r.find(1) == r.end());  // no route to self
}

WP_TEST(spf, cheaper_detour_beats_the_direct_link) {
  // The direct 1-3 link costs 10, the path through 2 costs 2.
  const Graph g = undirected({{1, 2, 1}, {2, 3, 1}, {1, 3, 10}});
  const RoutingTable r = shortest_paths(g, 1);
  CHECK_EQ(r.at(3).cost, 2u);
  CHECK_EQ(r.at(3).next_hops.size(), std::size_t{1});
  CHECK_EQ(r.at(3).next_hops[0], 2u);
}

WP_TEST(spf, equal_cost_multipath_is_reported) {
  //      2
  //    /   \        both branches cost 2
  //  1       4
  //    \   /
  //      3
  const Graph g = undirected({{1, 2, 1}, {1, 3, 1}, {2, 4, 1}, {3, 4, 1}});
  const RoutingTable r = shortest_paths(g, 1);
  CHECK_EQ(r.at(4).cost, 2u);
  CHECK_EQ(r.at(4).next_hops.size(), std::size_t{2});
  CHECK_EQ(r.at(4).next_hops[0], 2u);
  CHECK_EQ(r.at(4).next_hops[1], 3u);
  CHECK_EQ(r.at(4).parents.size(), std::size_t{2});
}

WP_TEST(spf, equal_cost_paths_propagate_beyond_the_split) {
  // Both halves of the diamond cost the same, and node 5 hangs off node 4, so
  // the two first hops must survive one more level of the computation.
  const Graph g = undirected(
      {{1, 2, 1}, {1, 3, 1}, {2, 4, 1}, {3, 4, 1}, {4, 5, 1}});
  const RoutingTable r = shortest_paths(g, 1);
  CHECK_EQ(r.at(5).cost, 3u);
  CHECK_EQ(r.at(5).next_hops.size(), std::size_t{2});
}

WP_TEST(spf, unreachable_nodes_are_absent) {
  Graph g = undirected({{1, 2, 1}});
  g.add_edge(9, 8, 1);
  g.add_edge(8, 9, 1);
  const RoutingTable r = shortest_paths(g, 1);
  CHECK_EQ(r.size(), std::size_t{1});
  CHECK_TRUE(r.find(8) == r.end());
  CHECK_TRUE(r.find(9) == r.end());
}

WP_TEST(spf, unknown_root_yields_an_empty_table) {
  const Graph g = undirected({{1, 2, 1}});
  CHECK_TRUE(shortest_paths(g, 77).empty());
}

WP_TEST(spf, one_way_link_is_not_used) {
  Graph g;
  g.add_edge(1, 2, 1);
  g.add_edge(2, 1, 1);
  g.add_edge(2, 3, 1);  // 3 never advertises the link back to 2
  const RoutingTable direct = shortest_paths(g, 1);
  CHECK_TRUE(direct.find(3) != direct.end());

  const RoutingTable checked = shortest_paths(g.bidirectional(), 1);
  CHECK_TRUE(checked.find(3) == checked.end());
}

WP_TEST(spf, dot_output_names_every_edge) {
  const Graph g = undirected({{1, 2, 7}});
  const std::string dot = topology_to_dot(g, "t");
  CHECK_TRUE(dot.find("n1 -- n2") != std::string::npos);
  CHECK_TRUE(dot.find("label=\"7\"") != std::string::npos);

  const std::string tree = spt_to_dot(1, shortest_paths(g, 1), "spt");
  CHECK_TRUE(tree.find("n1 -> n2") != std::string::npos);
}

WP_TEST(spf, dot_marks_a_one_sided_edge) {
  Graph g;
  g.add_edge(5, 1, 3);  // only one direction, and the peer id is the smaller
  const std::string dot = topology_to_dot(g, "t");
  CHECK_TRUE(dot.find("style=dashed") != std::string::npos);
}
