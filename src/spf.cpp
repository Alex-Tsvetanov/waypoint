#include "waypoint/spf.hpp"

#include <algorithm>
#include <cstdint>
#include <queue>
#include <sstream>
#include <unordered_map>

namespace waypoint {
namespace {

void insert_sorted_unique(std::vector<NodeId>& v, NodeId n) {
  auto it = std::lower_bound(v.begin(), v.end(), n);
  if (it == v.end() || *it != n) v.insert(it, n);
}

void merge_sorted_unique(std::vector<NodeId>& into,
                         const std::vector<NodeId>& from) {
  for (NodeId n : from) insert_sorted_unique(into, n);
}

}  // namespace

void Graph::add_edge(NodeId from, NodeId to, Cost cost) {
  AdjacencyList& list = adj[from];
  adj.try_emplace(to);  // the far end exists even with no outgoing edges
  for (Adjacency& a : list) {
    if (a.peer == to) {
      a.cost = cost;
      return;
    }
  }
  list.push_back(Adjacency{to, cost});
  std::sort(list.begin(), list.end(),
            [](const Adjacency& x, const Adjacency& y) { return x.peer < y.peer; });
}

bool Graph::has_edge(NodeId from, NodeId to) const {
  const auto it = adj.find(from);
  if (it == adj.end()) return false;
  for (const Adjacency& a : it->second) {
    if (a.peer == to) return true;
  }
  return false;
}

std::vector<NodeId> Graph::nodes() const {
  std::vector<NodeId> out;
  out.reserve(adj.size());
  for (const auto& [id, _] : adj) out.push_back(id);
  return out;
}

std::size_t Graph::edge_count() const {
  std::size_t n = 0;
  for (const auto& [id, list] : adj) {
    (void)id;
    n += list.size();
  }
  return n;
}

Graph Graph::bidirectional() const {
  Graph out;
  for (const auto& [from, list] : adj) {
    out.adj.try_emplace(from);
    for (const Adjacency& a : list) {
      if (has_edge(a.peer, from)) out.add_edge(from, a.peer, a.cost);
    }
  }
  return out;
}

RoutingTable shortest_paths(const Graph& graph, NodeId root) {
  RoutingTable table;
  if (graph.adj.find(root) == graph.adj.end()) return table;

  // Distances are accumulated in 64 bits so that a chain of large link costs
  // cannot silently wrap the 32 bit metric.
  std::unordered_map<NodeId, std::uint64_t> dist;
  std::unordered_map<NodeId, std::vector<NodeId>> next_hops;
  std::unordered_map<NodeId, std::vector<NodeId>> parents;

  using Item = std::pair<std::uint64_t, NodeId>;
  // Ties break on node id, so the traversal order does not depend on the
  // container's internal layout. Reproducibility starts here.
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;

  dist[root] = 0;
  queue.push({0, root});

  while (!queue.empty()) {
    const auto [d, u] = queue.top();
    queue.pop();
    const auto du = dist.find(u);
    if (du == dist.end() || d > du->second) continue;  // stale entry

    const auto adj_it = graph.adj.find(u);
    if (adj_it == graph.adj.end()) continue;

    for (const Adjacency& edge : adj_it->second) {
      const NodeId v = edge.peer;
      if (v == root) continue;
      const std::uint64_t nd = d + edge.cost;
      const auto dv = dist.find(v);
      if (dv == dist.end() || nd < dv->second) {
        dist[v] = nd;
        parents[v] = {u};
        next_hops[v] = (u == root) ? std::vector<NodeId>{v} : next_hops[u];
        queue.push({nd, v});
      } else if (nd == dv->second) {
        // Equal cost path: remember the extra parent and merge its first hops.
        insert_sorted_unique(parents[v], u);
        if (u == root) {
          insert_sorted_unique(next_hops[v], v);
        } else {
          merge_sorted_unique(next_hops[v], next_hops[u]);
        }
      }
    }
  }

  for (const auto& [node, d] : dist) {
    if (node == root) continue;
    Route r;
    r.cost = static_cast<Cost>(std::min<std::uint64_t>(d, kInfCost));
    r.next_hops = next_hops[node];
    r.parents = parents[node];
    std::sort(r.parents.begin(), r.parents.end());
    table.emplace(node, std::move(r));
  }
  return table;
}

std::string topology_to_dot(const Graph& graph, const std::string& name) {
  std::ostringstream out;
  out << "graph " << name << " {\n";
  out << "  layout=neato;\n  overlap=false;\n  node [shape=circle];\n";
  for (const NodeId n : graph.nodes()) out << "  n" << n << " [label=\"" << n << "\"];\n";
  for (const auto& [from, list] : graph.adj) {
    for (const Adjacency& a : list) {
      if (a.peer < from) continue;  // draw each undirected pair once
      const bool both = graph.has_edge(a.peer, from);
      out << "  n" << from << " -- n" << a.peer << " [label=\"" << a.cost << "\"";
      if (!both) out << ", style=dashed, color=red";
      out << "];\n";
    }
    // One sided edges whose reverse is missing and whose peer id is smaller
    // would be skipped by the loop above, so emit them here.
    for (const Adjacency& a : list) {
      if (a.peer >= from) continue;
      if (graph.has_edge(a.peer, from)) continue;
      out << "  n" << a.peer << " -- n" << from << " [label=\"" << a.cost
          << "\", style=dashed, color=red];\n";
    }
  }
  out << "}\n";
  return out.str();
}

std::string spt_to_dot(NodeId root, const RoutingTable& routes,
                       const std::string& name) {
  std::ostringstream out;
  out << "digraph " << name << " {\n";
  out << "  rankdir=LR;\n  node [shape=circle];\n";
  out << "  n" << root << " [label=\"" << root << "\", style=filled, fillcolor=lightblue];\n";
  for (const auto& [dest, route] : routes) {
    out << "  n" << dest << " [label=\"" << dest << "\\ncost " << route.cost << "\"];\n";
  }
  for (const auto& [dest, route] : routes) {
    for (const NodeId parent : route.parents) {
      out << "  n" << parent << " -> n" << dest;
      if (route.next_hops.size() > 1) out << " [color=darkgreen]";
      out << ";\n";
    }
  }
  out << "}\n";
  return out.str();
}

}  // namespace waypoint
