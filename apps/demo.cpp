// Waypoint demonstration: bring a twelve node network up, fail one link at a
// scheduled virtual time, and print exactly when each router noticed, when it
// recomputed, and when the network stopped looping.
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "waypoint/sim.hpp"

using namespace waypoint;

namespace {

std::string seconds(Micros t) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f s", static_cast<double>(t) / 1e6);
  return buffer;
}

bool write_file(const std::string& path, const std::string& content) {
  std::ofstream out(path, std::ios::binary);
  if (!out) return false;
  out << content;
  return static_cast<bool>(out);
}

void rule(const char* title) {
  std::printf("\n%s\n", title);
  std::printf("--------------------------------------------------------------\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string out_dir = "dot";
  std::uint64_t seed = 20260819;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
      out_dir = argv[++i];
    } else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      seed = std::strtoull(argv[++i], nullptr, 10);
    } else {
      std::fprintf(stderr, "usage: waypoint-demo [--out DIR] [--seed N]\n");
      return 2;
    }
  }

  // Twelve nodes in a ring. The ring was chosen after measuring: when a link
  // in a ring fails, the nodes on the far side of the break have to send in the
  // opposite direction from before, so for a short interval one end of an edge
  // has switched and the other has not. That is a transient forwarding loop,
  // and the ring is the smallest topology here that produces one. A grid mesh
  // of the same size never does, because the alternative path leaves every
  // remaining edge pointing the way it already pointed.
  const Topology topology = topo::ring(12);

  Simulator::Params params;
  params.router.hello_interval = 1 * kSec;
  params.router.dead_interval = 4 * kSec;
  params.router.rxmt_interval = 1 * kSec;
  params.router.spf_delay = 100 * kMs;
  params.seed = seed;
  params.detect_loops = true;
  // Routers put new routes into force at slightly different moments. Without
  // that spread every table changes on the same tick and a transient loop is
  // impossible by construction, which would be a property of the model rather
  // than of the protocol.
  params.install_delay_spread = 200 * kMs;

  Simulator sim(topology, params);

  std::printf("Waypoint, link state routing with a deterministic simulator\n");
  std::printf("==============================================================\n");
  std::printf("topology        %s, %zu nodes, %zu links\n", topology.name.c_str(),
              topology.nodes.size(), topology.links.size());
  std::printf("diameter        %zu hops, average degree %.2f\n", topology.diameter(),
              topology.average_degree());
  std::printf("hello / dead    %s / %s\n", seconds(params.router.hello_interval).c_str(),
              seconds(params.router.dead_interval).c_str());
  std::printf("SPF hold down   %s\n", seconds(params.router.spf_delay).c_str());
  std::printf("seed            %llu\n", static_cast<unsigned long long>(seed));

  rule("Phase 1: cold start");
  sim.run_until(30 * kSec);
  if (!sim.converged()) {
    std::fprintf(stderr, "the network did not converge from cold; aborting\n");
    return 1;
  }
  std::printf("all %zu routers agree with Dijkstra on the true topology at %s\n",
              topology.nodes.size(), seconds(sim.last_converged_at()).c_str());
  std::printf("start up cost   %llu messages, %llu bytes\n",
              static_cast<unsigned long long>(sim.metrics().messages_sent),
              static_cast<unsigned long long>(sim.metrics().bytes_sent));
  const RoutingTable before = sim.router(1).rib();
  std::printf("node 1 to node 12: cost %u via", before.at(12).cost);
  for (const NodeId h : before.at(12).next_hops) std::printf(" %u", h);
  std::printf("\n");

  // Any link of a ring is equivalent to any other by symmetry, so the choice
  // of 6 to 7 carries no information and needs none.
  const NodeId a = 6;
  const NodeId b = 7;
  const Micros failure = 40 * kSec;

  sim.mark_baseline();
  sim.fail_link(failure, a, b);
  sim.run_until(failure + 40 * kSec);

  rule("Phase 2: link 6 to 7 fails");
  std::printf("failure injected at %s of virtual time\n", seconds(failure).c_str());

  std::map<NodeId, Micros> noticed;
  std::map<NodeId, Micros> recomputed;
  std::map<NodeId, Micros> installed;
  for (const LogEvent& e : sim.log()) {
    if (e.time < failure) continue;
    switch (e.kind) {
      case EventKind::NeighborState:
        // The two ends of the failed link notice by timing the adjacency out.
        if (e.b < static_cast<std::uint32_t>(NeighborState::Full) &&
            e.a == static_cast<std::uint32_t>(NeighborState::Full)) {
          noticed.try_emplace(e.node, e.time);
        }
        break;
      case EventKind::LsaInstalled:
        // Everyone else notices when the new advertisement reaches them.
        noticed.try_emplace(e.node, e.time);
        break;
      case EventKind::SpfComputed:
        recomputed.try_emplace(e.node, e.time);
        break;
      case EventKind::RouteInstalled:
        if (e.a > 0) installed.try_emplace(e.node, e.time);
        break;
      default:
        break;
    }
  }

  std::printf("\n%-6s %-16s %-16s %-16s\n", "node", "noticed", "recomputed", "installed");
  for (const NodeId n : topology.nodes) {
    auto at = [&](const std::map<NodeId, Micros>& m) {
      const auto it = m.find(n);
      return it == m.end() ? std::string("-") : seconds(it->second - failure);
    };
    std::printf("%-6u %-16s %-16s %-16s\n", n, at(noticed).c_str(),
                at(recomputed).c_str(), at(installed).c_str());
  }
  std::printf("(times are relative to the failure)\n");

  rule("Phase 3: what it cost");
  if (sim.converged() && sim.last_converged_at() >= 0) {
    std::printf("reconverged     %s after the failure\n",
                seconds(sim.last_converged_at() - failure).c_str());
  } else {
    std::printf("reconverged     NOT REACHED within the run\n");
  }
  if (sim.loop_episodes().empty()) {
    std::printf("transient loops none observed\n");
  } else {
    for (const LoopEpisode& l : sim.loop_episodes()) {
      std::printf("transient loop  from %s to %s, lasting %s, %zu source and "
                  "destination pairs affected\n",
                  seconds(l.start - failure).c_str(),
                  l.end < 0 ? "the end of the run" : seconds(l.end - failure).c_str(),
                  l.duration() < 0 ? "unfinished" : seconds(l.duration()).c_str(),
                  l.peak_pairs);
    }
  }
  const SimMetrics& m = sim.metrics();
  std::printf("flooding        %llu update messages, %llu bytes\n",
              static_cast<unsigned long long>(m.update_messages),
              static_cast<unsigned long long>(m.update_bytes));
  std::printf("all traffic     %llu messages, %llu bytes, %llu dropped on the "
              "failed link\n",
              static_cast<unsigned long long>(m.messages_sent),
              static_cast<unsigned long long>(m.bytes_sent),
              static_cast<unsigned long long>(m.messages_dropped));
  std::printf("route churn     %llu changed destinations across all routers\n",
              static_cast<unsigned long long>(m.route_changes));
  std::printf("computations    %llu shortest path runs\n",
              static_cast<unsigned long long>(m.spf_runs));

  const RoutingTable after = sim.router(1).rib();
  std::printf("node 1 to node 12: cost %u via", after.at(12).cost);
  for (const NodeId h : after.at(12).next_hops) std::printf(" %u", h);
  std::printf("\n");

  rule("Phase 4: reproducibility");
  Simulator repeat(topology, params);
  repeat.run_until(30 * kSec);
  repeat.mark_baseline();
  repeat.fail_link(failure, a, b);
  repeat.run_until(failure + 40 * kSec);
  const bool identical = repeat.log_digest() == sim.log_digest();
  std::printf("event log digest %016llx, repeated run %016llx: %s\n",
              static_cast<unsigned long long>(sim.log_digest()),
              static_cast<unsigned long long>(repeat.log_digest()),
              identical ? "identical" : "DIFFERENT");
  if (!identical) {
    std::fprintf(stderr, "the run is not reproducible; that is a defect\n");
    return 1;
  }

  rule("Output files");
  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  const std::string prefix = out_dir + "/";
  struct Artifact {
    std::string path;
    std::string content;
  };
  const std::vector<Artifact> artifacts = {
      {prefix + "topology.txt", topology.to_text()},
      {prefix + "topology-before.dot", topology_to_dot(topology.graph(), "before")},
      {prefix + "topology-after.dot", topology_to_dot(sim.truth(), "after")},
      {prefix + "spt-node1-before.dot", spt_to_dot(1, before, "spt_before")},
      {prefix + "spt-node1-after.dot", spt_to_dot(1, after, "spt_after")},
      {prefix + "timeline.txt", sim.timeline_text()},
  };
  for (const Artifact& artifact : artifacts) {
    if (write_file(artifact.path, artifact.content)) {
      std::printf("wrote %s\n", artifact.path.c_str());
    } else {
      std::fprintf(stderr,
                   "could not write %s; create the directory first, for example "
                   "mkdir %s\n",
                   artifact.path.c_str(), out_dir.c_str());
      return 1;
    }
  }
  std::printf("\nrender with: dot -Tpng %stopology-after.dot -o after.png\n",
              prefix.c_str());
  return 0;
}
