#include "check.hpp"
#include "waypoint/topology.hpp"

using namespace waypoint;

WP_TEST(topology, ring_shape_and_diameter) {
  const Topology r = topo::ring(6);
  CHECK_EQ(r.nodes.size(), std::size_t{6});
  CHECK_EQ(r.links.size(), std::size_t{6});
  CHECK_EQ(r.diameter(), std::size_t{3});
  CHECK_TRUE(r.average_degree() == 2.0);

  const Topology r12 = topo::ring(12);
  CHECK_EQ(r12.links.size(), std::size_t{12});
  CHECK_EQ(r12.diameter(), std::size_t{6});
}

WP_TEST(topology, grid_mesh_shape) {
  const Topology m = topo::grid_mesh(3, 4);
  CHECK_EQ(m.nodes.size(), std::size_t{12});
  // A rows x cols grid has rows*(cols-1) horizontal and (rows-1)*cols vertical.
  CHECK_EQ(m.links.size(), std::size_t{3 * 3 + 2 * 4});
  CHECK_EQ(m.diameter(), std::size_t{5});
}

WP_TEST(topology, full_mesh_shape) {
  const Topology f = topo::full_mesh(5);
  CHECK_EQ(f.links.size(), std::size_t{10});
  CHECK_EQ(f.diameter(), std::size_t{1});
  CHECK_TRUE(f.average_degree() == 4.0);
}

WP_TEST(topology, fat_tree_shape) {
  const Topology t = topo::fat_tree(4);
  // 4 core, 4 pods of 2 aggregation plus 2 edge switches.
  CHECK_EQ(t.nodes.size(), std::size_t{4 + 4 * 4});
  // Each pod: 2 aggregation x 2 edge, plus 2 aggregation x 2 core uplinks.
  CHECK_EQ(t.links.size(), std::size_t{4 * (2 * 2 + 2 * 2)});
  CHECK_EQ(t.diameter(), std::size_t{4});
  // An odd or degenerate arity has no fat tree, and must not fake one.
  CHECK_TRUE(topo::fat_tree(3).nodes.empty());
}

WP_TEST(topology, text_round_trip_preserves_everything) {
  Topology original = topo::grid_mesh(2, 3);
  original.links[0].cost = 17;
  original.links[0].delay = 5 * kMs;
  original.links[0].bandwidth_bps = 1234567;

  std::string error;
  const auto back = Topology::parse(original.to_text(), error);
  CHECK_TRUE(back.has_value());
  CHECK_TRUE(error.empty());
  CHECK_TRUE(back->name == original.name);
  CHECK_TRUE(back->nodes == original.nodes);
  CHECK_TRUE(back->links == original.links);
}

WP_TEST(topology, endpoints_named_only_on_a_link_still_become_nodes) {
  std::string error;
  const auto t = Topology::parse("link 4 9 cost 3\n# a comment\n", error);
  CHECK_TRUE(t.has_value());
  CHECK_EQ(t->nodes.size(), std::size_t{2});
  CHECK_EQ(t->nodes[0], 4u);
  CHECK_EQ(t->nodes[1], 9u);
  CHECK_EQ(t->links.at(0).cost, 3u);
}

WP_TEST(topology, malformed_input_is_refused_with_a_line_number) {
  std::string error;
  CHECK_FALSE(Topology::parse("node 1\nwibble 2\n", error).has_value());
  CHECK_TRUE(error.find("line 2") != std::string::npos);

  CHECK_FALSE(Topology::parse("link 1 1\n", error).has_value());
  CHECK_FALSE(Topology::parse("link 1 2 cost 0\n", error).has_value());
  CHECK_FALSE(Topology::parse("link 1 2 bw 0\n", error).has_value());
  CHECK_FALSE(Topology::parse("link 1 2 cost\n", error).has_value());
  CHECK_FALSE(Topology::parse("link 1 2 wibble 3\n", error).has_value());
  CHECK_FALSE(Topology::parse("node x\n", error).has_value());
}

WP_TEST(topology, graph_view_is_undirected) {
  const Topology r = topo::ring(4);
  const Graph g = r.graph();
  CHECK_TRUE(g.has_edge(1, 2));
  CHECK_TRUE(g.has_edge(2, 1));
  CHECK_TRUE(g.has_edge(4, 1));
  CHECK_EQ(g.edge_count(), std::size_t{8});
  // Nothing is one sided, so the usable view is the same graph.
  CHECK_EQ(g.bidirectional().edge_count(), std::size_t{8});
}
