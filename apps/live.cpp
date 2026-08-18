// Live mode. One process is one router, talking to its neighbours over UDP.
// The Router object below is the same class the simulator drives; the only
// difference is which Env it was handed.
//
//   waypoint-live --topology t.txt --id 1
//   waypoint-live --topology t.txt --id 2
//
// The two processes discover each other, form an adjacency, exchange
// databases and install routes, exactly as they do under the virtual clock.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

#include "waypoint/topology.hpp"
#include "waypoint/udp.hpp"

using namespace waypoint;

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: waypoint-live --topology FILE --id N [--host 127.0.0.1]\n"
               "                     [--base-port 25000] [--seconds 30]\n"
               "                     [--hello MS] [--dead MS]\n"
               "Each router listens on base-port + its own id.\n");
}

void print_state(const Router& router, const UdpEnv& env, Micros elapsed) {
  std::printf("\n[%6.1f s] router %u\n", static_cast<double>(elapsed) / 1e6, router.id());
  std::printf("  adjacencies:");
  for (const auto& [peer, cost] : router.links()) {
    (void)cost;
    std::printf("  %u=%s", peer, to_string(router.state_of(peer)));
  }
  std::printf("\n  database: %zu advertisements, %u computations, %u originations\n",
              router.lsdb().size(), router.spf_runs(), router.originations());
  std::printf("  routes:");
  if (router.rib().empty()) std::printf("  none yet");
  for (const auto& [destination, route] : router.rib()) {
    std::printf("  %u(cost %u via", destination, route.cost);
    for (const NodeId hop : route.next_hops) std::printf(" %u", hop);
    std::printf(")");
  }
  std::printf("\n  udp: %llu sent, %llu received, %llu failed\n",
              static_cast<unsigned long long>(env.datagrams_sent()),
              static_cast<unsigned long long>(env.datagrams_received()),
              static_cast<unsigned long long>(env.send_failures()));
  std::fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
  std::string topology_path;
  std::string host = "127.0.0.1";
  long id = -1;
  long base_port = 25000;
  long seconds = 30;
  long hello_ms = 1000;
  long dead_ms = 4000;

  for (int i = 1; i < argc; ++i) {
    const char* flag = argv[i];
    const bool has_value = i + 1 < argc;
    if (std::strcmp(flag, "--topology") == 0 && has_value) {
      topology_path = argv[++i];
    } else if (std::strcmp(flag, "--id") == 0 && has_value) {
      id = std::strtol(argv[++i], nullptr, 10);
    } else if (std::strcmp(flag, "--host") == 0 && has_value) {
      host = argv[++i];
    } else if (std::strcmp(flag, "--base-port") == 0 && has_value) {
      base_port = std::strtol(argv[++i], nullptr, 10);
    } else if (std::strcmp(flag, "--seconds") == 0 && has_value) {
      seconds = std::strtol(argv[++i], nullptr, 10);
    } else if (std::strcmp(flag, "--hello") == 0 && has_value) {
      hello_ms = std::strtol(argv[++i], nullptr, 10);
    } else if (std::strcmp(flag, "--dead") == 0 && has_value) {
      dead_ms = std::strtol(argv[++i], nullptr, 10);
    } else {
      usage();
      return 2;
    }
  }
  if (topology_path.empty() || id < 0) {
    usage();
    return 2;
  }

  std::string error;
  const auto topology = Topology::load(topology_path, error);
  if (!topology) {
    std::fprintf(stderr, "topology: %s\n", error.c_str());
    return 1;
  }

  const NodeId self = static_cast<NodeId>(id);
  std::map<NodeId, Endpoint> peers;
  std::map<NodeId, Cost> costs;
  for (const TopologyLink& l : topology->links) {
    if (l.a == self) costs[l.b] = l.cost;
    if (l.b == self) costs[l.a] = l.cost;
  }
  if (costs.empty()) {
    std::fprintf(stderr, "node %u has no links in %s\n", self, topology_path.c_str());
    return 1;
  }
  for (const auto& [peer, cost] : costs) {
    (void)cost;
    peers[peer] = Endpoint{host, static_cast<std::uint16_t>(base_port + peer)};
  }

  UdpSocket socket;
  if (!socket.open(static_cast<std::uint16_t>(base_port + self), error)) {
    std::fprintf(stderr, "cannot listen: %s\n", error.c_str());
    return 1;
  }

  UdpEnv env(peers, socket);
  Router::Config config;
  config.id = self;
  config.hello_interval = hello_ms * kMs;
  config.dead_interval = dead_ms * kMs;
  config.rxmt_interval = hello_ms * kMs;
  config.spf_delay = 100 * kMs;

  Router router(config, env);
  for (const auto& [peer, cost] : costs) router.add_link(peer, cost);

  std::printf("waypoint-live: router %u on udp port %u, %zu interfaces\n", self,
              socket.local_port(), costs.size());
  for (const auto& [peer, endpoint] : peers) {
    std::printf("  interface to %u at %s:%u cost %u\n", peer, endpoint.host.c_str(),
                endpoint.port, costs.at(peer));
  }
  std::fflush(stdout);

  router.start();
  const Micros deadline = static_cast<Micros>(seconds) * kSec;
  const Micros start = env.now();
  Micros next_report = 0;
  while (env.now() - start < deadline) {
    env.pump(router, 200 * kMs);
    if (env.now() - start >= next_report) {
      print_state(router, env, env.now() - start);
      next_report += 5 * kSec;
    }
  }
  print_state(router, env, env.now() - start);
  return 0;
}
