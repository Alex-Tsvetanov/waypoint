#include "waypoint/analysis.hpp"

#include <algorithm>

namespace waypoint {

std::vector<std::pair<NodeId, NodeId>> find_forwarding_loops(const RibView& ribs) {
  std::vector<std::pair<NodeId, NodeId>> loops;
  const std::size_t limit = ribs.size() + 1;

  for (const auto& [source, table] : ribs) {
    if (table == nullptr) continue;
    for (const auto& [destination, route] : *table) {
      (void)route;
      NodeId current = source;
      std::set<NodeId> visited{source};
      for (std::size_t step = 0; step < limit; ++step) {
        const auto holder = ribs.find(current);
        if (holder == ribs.end() || holder->second == nullptr) break;
        const auto entry = holder->second->find(destination);
        if (entry == holder->second->end()) break;  // black hole, not a loop
        if (entry->second.next_hops.empty()) break;
        const NodeId next = entry->second.next_hops.front();
        if (next == destination) break;  // delivered
        if (!visited.insert(next).second) {
          loops.emplace_back(source, destination);
          break;
        }
        current = next;
      }
    }
  }
  return loops;
}

bool ribs_match_truth(const RibView& ribs, const Graph& truth,
                      const std::set<NodeId>& skip) {
  for (const auto& [node, table] : ribs) {
    if (skip.find(node) != skip.end()) continue;
    if (table == nullptr) return false;
    const RoutingTable expected = shortest_paths(truth, node);
    if (expected.size() != table->size()) return false;
    for (const auto& [destination, route] : expected) {
      const auto held = table->find(destination);
      if (held == table->end()) return false;
      if (!(held->second == route)) return false;
    }
  }
  return true;
}

std::vector<std::pair<NodeId, NodeId>> find_black_holes(const RibView& ribs,
                                                        const Graph& truth,
                                                        const std::set<NodeId>& skip) {
  std::vector<std::pair<NodeId, NodeId>> holes;
  for (const auto& [node, table] : ribs) {
    if (skip.find(node) != skip.end() || table == nullptr) continue;
    const RoutingTable expected = shortest_paths(truth, node);
    for (const auto& [destination, route] : expected) {
      (void)route;
      if (skip.find(destination) != skip.end()) continue;
      if (table->find(destination) == table->end()) holes.emplace_back(node, destination);
    }
  }
  return holes;
}

}  // namespace waypoint
