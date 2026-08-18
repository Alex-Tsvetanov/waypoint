// Topology description: what the simulator is handed, and what a file on disk
// contains.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "waypoint/spf.hpp"
#include "waypoint/types.hpp"

namespace waypoint {

struct TopologyLink {
  NodeId a = kNoNode;
  NodeId b = kNoNode;
  Cost cost = 1;
  Micros delay = 1 * kMs;             // one way propagation delay
  std::uint64_t bandwidth_bps = 100 * 1000 * 1000;

  friend bool operator==(const TopologyLink&, const TopologyLink&) = default;
};

struct Topology {
  std::string name = "topology";
  std::vector<NodeId> nodes;
  std::vector<TopologyLink> links;

  // Undirected view, both directions present. This is the ground truth the
  // measured routing tables are checked against.
  Graph graph() const;

  // Longest shortest path in hops. Diameter drives how long an advertisement
  // needs to reach the far side of the network, so it is reported with every
  // convergence result.
  std::size_t diameter() const;
  double average_degree() const;

  std::string to_text() const;

  // Returns nullopt and fills `error` on the first line that does not parse.
  // A topology that half loaded would produce a measurement of something
  // nobody described.
  static std::optional<Topology> parse(std::string_view text, std::string& error);
  static std::optional<Topology> load(const std::string& path, std::string& error);
};

namespace topo {

// Ring of `n` nodes, ids 1..n. Largest diameter for a given node count among
// the connected topologies here, so it is the pessimistic case.
Topology ring(std::size_t n, Cost cost = 1);

// Rectangular mesh, ids laid out row by row.
Topology grid_mesh(std::size_t rows, std::size_t cols, Cost cost = 1);

// Every node connected to every other. Diameter one, maximum flooding load.
Topology full_mesh(std::size_t n, Cost cost = 1);

// k-ary fat tree as used in data centre fabrics: (k/2)^2 core switches, k pods
// of k/2 aggregation and k/2 edge switches. k must be even and at least two.
Topology fat_tree(std::size_t k, Cost cost = 1);

}  // namespace topo

}  // namespace waypoint
