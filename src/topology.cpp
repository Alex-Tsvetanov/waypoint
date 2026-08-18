#include "waypoint/topology.hpp"

#include <algorithm>
#include <deque>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace waypoint {
namespace {

std::vector<std::string> split_tokens(const std::string& line) {
  std::vector<std::string> out;
  std::istringstream in(line);
  std::string token;
  while (in >> token) {
    if (!token.empty() && token[0] == '#') break;  // trailing comment
    out.push_back(token);
  }
  return out;
}

bool parse_u64(const std::string& text, std::uint64_t& out) {
  if (text.empty()) return false;
  std::uint64_t value = 0;
  for (const char c : text) {
    if (c < '0' || c > '9') return false;
    if (value > (~std::uint64_t{0} - static_cast<std::uint64_t>(c - '0')) / 10) return false;
    value = value * 10 + static_cast<std::uint64_t>(c - '0');
  }
  out = value;
  return true;
}

}  // namespace

Graph Topology::graph() const {
  Graph g;
  for (const NodeId n : nodes) g.adj.try_emplace(n);
  for (const TopologyLink& l : links) {
    g.add_edge(l.a, l.b, l.cost);
    g.add_edge(l.b, l.a, l.cost);
  }
  return g;
}

std::size_t Topology::diameter() const {
  std::map<NodeId, std::vector<NodeId>> adj;
  for (const NodeId n : nodes) adj.try_emplace(n);
  for (const TopologyLink& l : links) {
    adj[l.a].push_back(l.b);
    adj[l.b].push_back(l.a);
  }

  std::size_t worst = 0;
  for (const NodeId start : nodes) {
    std::map<NodeId, std::size_t> depth{{start, 0}};
    std::deque<NodeId> queue{start};
    while (!queue.empty()) {
      const NodeId u = queue.front();
      queue.pop_front();
      for (const NodeId v : adj[u]) {
        if (depth.find(v) != depth.end()) continue;
        depth[v] = depth[u] + 1;
        worst = std::max(worst, depth[v]);
        queue.push_back(v);
      }
    }
  }
  return worst;
}

double Topology::average_degree() const {
  if (nodes.empty()) return 0.0;
  return 2.0 * static_cast<double>(links.size()) / static_cast<double>(nodes.size());
}

std::string Topology::to_text() const {
  std::ostringstream out;
  out << "# waypoint topology\n";
  out << "name " << name << "\n";
  for (const NodeId n : nodes) out << "node " << n << "\n";
  for (const TopologyLink& l : links) {
    out << "link " << l.a << " " << l.b << " cost " << l.cost << " delay "
        << l.delay << " bw " << l.bandwidth_bps << "\n";
  }
  return out.str();
}

std::optional<Topology> Topology::parse(std::string_view text, std::string& error) {
  Topology t;
  std::set<NodeId> seen;
  std::istringstream in{std::string(text)};
  std::string line;
  int line_number = 0;

  auto bad = [&](const std::string& why) {
    error = "line " + std::to_string(line_number) + ": " + why;
    return std::nullopt;
  };

  while (std::getline(in, line)) {
    ++line_number;
    const std::vector<std::string> tok = split_tokens(line);
    if (tok.empty()) continue;

    if (tok[0] == "name") {
      if (tok.size() != 2) return bad("name takes one word");
      t.name = tok[1];
      continue;
    }
    if (tok[0] == "node") {
      std::uint64_t id = 0;
      if (tok.size() != 2 || !parse_u64(tok[1], id) || id > kNoNode - 1) {
        return bad("node takes one numeric id");
      }
      if (seen.insert(static_cast<NodeId>(id)).second) {
        t.nodes.push_back(static_cast<NodeId>(id));
      }
      continue;
    }
    if (tok[0] == "link") {
      if (tok.size() < 3) return bad("link takes two node ids");
      std::uint64_t a = 0;
      std::uint64_t b = 0;
      if (!parse_u64(tok[1], a) || !parse_u64(tok[2], b)) {
        return bad("link endpoints must be numeric");
      }
      if (a == b) return bad("a link may not join a node to itself");
      TopologyLink l;
      l.a = static_cast<NodeId>(a);
      l.b = static_cast<NodeId>(b);
      for (std::size_t i = 3; i + 1 < tok.size(); i += 2) {
        std::uint64_t value = 0;
        if (!parse_u64(tok[i + 1], value)) return bad("attribute value must be numeric");
        if (tok[i] == "cost") {
          // A zero cost link would break the ordering assumption the shortest
          // path computation relies on, so it is refused here rather than
          // producing a quietly wrong routing table later.
          if (value == 0 || value > kInfCost) return bad("cost must be between 1 and 2^32-2");
          l.cost = static_cast<Cost>(value);
        } else if (tok[i] == "delay") {
          l.delay = static_cast<Micros>(value);
        } else if (tok[i] == "bw") {
          if (value == 0) return bad("bandwidth must be greater than zero");
          l.bandwidth_bps = value;
        } else {
          return bad("unknown link attribute '" + tok[i] + "'");
        }
      }
      if ((tok.size() - 3) % 2 != 0) return bad("link attributes come in pairs");
      if (seen.insert(l.a).second) t.nodes.push_back(l.a);
      if (seen.insert(l.b).second) t.nodes.push_back(l.b);
      t.links.push_back(l);
      continue;
    }
    return bad("unknown directive '" + tok[0] + "'");
  }

  std::sort(t.nodes.begin(), t.nodes.end());
  error.clear();
  return t;
}

std::optional<Topology> Topology::load(const std::string& path, std::string& error) {
  std::ifstream file(path);
  if (!file) {
    error = "cannot open '" + path + "'";
    return std::nullopt;
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return parse(buffer.str(), error);
}

namespace topo {
namespace {

void add(Topology& t, NodeId a, NodeId b, Cost cost) {
  TopologyLink l;
  l.a = a;
  l.b = b;
  l.cost = cost;
  t.links.push_back(l);
}

void fill_nodes(Topology& t, std::size_t n) {
  for (std::size_t i = 1; i <= n; ++i) t.nodes.push_back(static_cast<NodeId>(i));
}

}  // namespace

Topology ring(std::size_t n, Cost cost) {
  Topology t;
  t.name = "ring" + std::to_string(n);
  if (n < 2) {
    fill_nodes(t, n);
    return t;
  }
  fill_nodes(t, n);
  for (std::size_t i = 1; i <= n; ++i) {
    const NodeId a = static_cast<NodeId>(i);
    const NodeId b = static_cast<NodeId>(i % n + 1);
    if (n == 2 && i == 2) break;  // two nodes share a single link
    add(t, a, b, cost);
  }
  return t;
}

Topology grid_mesh(std::size_t rows, std::size_t cols, Cost cost) {
  Topology t;
  t.name = "mesh" + std::to_string(rows) + "x" + std::to_string(cols);
  fill_nodes(t, rows * cols);
  auto id = [cols](std::size_t r, std::size_t c) {
    return static_cast<NodeId>(r * cols + c + 1);
  };
  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t c = 0; c < cols; ++c) {
      if (c + 1 < cols) add(t, id(r, c), id(r, c + 1), cost);
      if (r + 1 < rows) add(t, id(r, c), id(r + 1, c), cost);
    }
  }
  return t;
}

Topology full_mesh(std::size_t n, Cost cost) {
  Topology t;
  t.name = "full" + std::to_string(n);
  fill_nodes(t, n);
  for (std::size_t i = 1; i <= n; ++i) {
    for (std::size_t j = i + 1; j <= n; ++j) {
      add(t, static_cast<NodeId>(i), static_cast<NodeId>(j), cost);
    }
  }
  return t;
}

Topology fat_tree(std::size_t k, Cost cost) {
  Topology t;
  t.name = "fattree" + std::to_string(k);
  if (k < 2 || k % 2 != 0) return t;

  const std::size_t half = k / 2;
  const std::size_t cores = half * half;
  // Numbering: cores first, then per pod the aggregation switches followed by
  // the edge switches. Keeping it arithmetic rather than table driven means the
  // wiring below reads as the definition rather than as a lookup.
  const std::size_t per_pod = 2 * half;
  fill_nodes(t, cores + k * per_pod);

  auto core = [](std::size_t i) { return static_cast<NodeId>(i + 1); };
  auto agg = [cores, per_pod](std::size_t pod, std::size_t i) {
    return static_cast<NodeId>(cores + pod * per_pod + i + 1);
  };
  auto edge = [cores, per_pod, half](std::size_t pod, std::size_t i) {
    return static_cast<NodeId>(cores + pod * per_pod + half + i + 1);
  };

  for (std::size_t pod = 0; pod < k; ++pod) {
    for (std::size_t a = 0; a < half; ++a) {
      for (std::size_t e = 0; e < half; ++e) add(t, agg(pod, a), edge(pod, e), cost);
      for (std::size_t c = 0; c < half; ++c) add(t, core(a * half + c), agg(pod, a), cost);
    }
  }
  return t;
}

}  // namespace topo
}  // namespace waypoint
