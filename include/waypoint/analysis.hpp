// Post hoc analysis of routing state. These functions look only at routing
// tables and at the true topology, never at protocol internals, so the same
// analysis applies to a simulated run and to a live capture.
#pragma once

#include <map>
#include <set>
#include <utility>
#include <vector>

#include "waypoint/spf.hpp"
#include "waypoint/types.hpp"

namespace waypoint {

// Routing tables of every router, borrowed rather than copied.
using RibView = std::map<NodeId, const RoutingTable*>;

// Every (source, destination) pair whose forwarding path revisits a node.
// Following the first equal cost next hop is enough: a loop on any one of the
// equal cost branches is a loop.
std::vector<std::pair<NodeId, NodeId>> find_forwarding_loops(const RibView& ribs);

// True when every router listed, except those in `skip`, holds exactly the
// routing table that Dijkstra produces on the true topology. This is the
// definition of converged used throughout the measurements.
bool ribs_match_truth(const RibView& ribs, const Graph& truth,
                      const std::set<NodeId>& skip);

// Destinations for which a router has no route although the true topology
// still connects it. Useful for telling a black hole apart from a loop.
std::vector<std::pair<NodeId, NodeId>> find_black_holes(const RibView& ribs,
                                                        const Graph& truth,
                                                        const std::set<NodeId>& skip);

}  // namespace waypoint
