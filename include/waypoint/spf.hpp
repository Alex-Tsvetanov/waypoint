// Graph reconstruction and shortest path computation.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "waypoint/types.hpp"

namespace waypoint {

// Directed graph. A link state database yields a directed graph because each
// router advertises only its own side of every link; the undirected view is
// derived, not assumed.
struct Graph {
  std::map<NodeId, AdjacencyList> adj;

  void add_edge(NodeId from, NodeId to, Cost cost);
  bool has_edge(NodeId from, NodeId to) const;
  std::vector<NodeId> nodes() const;
  std::size_t edge_count() const;

  // RFC 2328 uses a link in the shortest path calculation only when both ends
  // advertise it. Dropping the one sided edges here is what makes a one way
  // link failure produce a correct, if slower, reconvergence instead of a
  // permanent black hole.
  Graph bidirectional() const;
};

struct Route {
  Cost cost = kInfCost;
  // Equal cost multipath: every first hop that lies on a shortest path.
  std::vector<NodeId> next_hops;
  // Predecessors on shortest paths, used to draw the shortest path graph.
  std::vector<NodeId> parents;

  friend bool operator==(const Route&, const Route&) = default;
};

// Ordered so that two routing tables compare and print deterministically.
using RoutingTable = std::map<NodeId, Route>;

// Dijkstra over `graph` rooted at `root`. The root itself is not present in
// the result: a router does not hold a route to itself.
RoutingTable shortest_paths(const Graph& graph, NodeId root);

// Graphviz output. `topology_to_dot` draws the graph, marking one sided edges
// so a one way failure is visible. `spt_to_dot` draws the shortest path graph
// rooted at `root`, which is a directed acyclic graph rather than a tree when
// equal cost paths exist.
std::string topology_to_dot(const Graph& graph, const std::string& name);
std::string spt_to_dot(NodeId root, const RoutingTable& routes,
                       const std::string& name);

}  // namespace waypoint
