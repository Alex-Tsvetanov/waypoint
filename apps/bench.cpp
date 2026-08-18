// Measurement harness. Every number the report contains comes out of this
// program. Timing is virtual, not wall clock, so the results do not depend on
// how busy the machine was; the only source of variation between repetitions
// is the seed, which sets the phase of each router's hello timer.
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "waypoint/sim.hpp"

using namespace waypoint;

namespace {

constexpr int kRepetitions = 10;
constexpr Micros kWarmUp = 40 * kSec;
constexpr Micros kObserve = 60 * kSec;
// Spread of the per router forwarding table installation delay used by the
// loop experiment.
constexpr Micros kInstallSpread = 200 * kMs;

struct Outcome {
  bool valid = false;
  Micros convergence = -1;
  std::uint64_t update_messages = 0;
  std::uint64_t update_bytes = 0;
  std::uint64_t flood_messages = 0;
  std::uint64_t flood_bytes = 0;
  std::uint64_t route_changes = 0;
  std::size_t loop_episodes = 0;
  Micros loop_duration = 0;
  std::uint64_t digest = 0;
};

enum class Failure { LinkBothWays, LinkOneWay, Node };

struct Scenario {
  Topology topology;
  Micros hello = 1 * kSec;
  Micros dead = 4 * kSec;
  Micros spf_delay = 100 * kMs;
  NodeId a = 1;
  NodeId b = 2;
  Failure failure = Failure::LinkBothWays;
  bool detect_loops = true;
  Micros install_spread = 0;
};

Outcome run_once(const Scenario& scenario, std::uint64_t seed) {
  Simulator::Params params;
  params.router.hello_interval = scenario.hello;
  params.router.dead_interval = scenario.dead;
  params.router.rxmt_interval = scenario.hello;
  params.router.spf_delay = scenario.spf_delay;
  params.seed = seed;
  params.detect_loops = scenario.detect_loops;
  params.install_delay_spread = scenario.install_spread;

  Simulator sim(scenario.topology, params);
  sim.run_until(kWarmUp);

  Outcome out;
  // Validity condition one: the network must be converged before the failure,
  // otherwise what follows measures the tail of the start up, not a recovery.
  if (!sim.converged()) return out;

  sim.mark_baseline();
  const Micros failure_time = kWarmUp + 1 * kSec;
  switch (scenario.failure) {
    case Failure::LinkBothWays: sim.fail_link(failure_time, scenario.a, scenario.b); break;
    case Failure::LinkOneWay:
      sim.fail_link(failure_time, scenario.a, scenario.b, /*one_way=*/true);
      break;
    case Failure::Node: sim.fail_node(failure_time, scenario.a); break;
  }
  sim.run_until(failure_time + kObserve);

  // Validity condition two: it must have converged again by the end of the
  // observation window, or the number would be a lower bound and not a result.
  if (!sim.converged() || sim.last_converged_at() < failure_time) return out;

  out.valid = true;
  out.convergence = sim.last_converged_at() - failure_time;
  out.update_messages = sim.metrics().update_messages;
  out.update_bytes = sim.metrics().update_bytes;
  out.flood_messages = sim.metrics().flood_messages;
  out.flood_bytes = sim.metrics().flood_bytes;
  out.route_changes = sim.metrics().route_changes;
  out.loop_episodes = sim.loop_episodes().size();
  for (const LoopEpisode& l : sim.loop_episodes()) {
    if (l.duration() > 0) out.loop_duration += l.duration();
  }
  out.digest = sim.log_digest();
  return out;
}

struct Summary {
  int valid = 0;
  int rejected = 0;
  double mean_ms = 0;
  Micros min_us = 0;
  Micros max_us = 0;
  double mean_update_messages = 0;
  double mean_update_bytes = 0;
  double mean_route_changes = 0;
  double mean_loops = 0;
  double mean_loop_ms = 0;
  bool reproducible = true;
};

Summary repeat(const Scenario& scenario) {
  Summary s;
  std::vector<Micros> times;
  for (int i = 0; i < kRepetitions; ++i) {
    const std::uint64_t seed = 1000 + static_cast<std::uint64_t>(i);
    const Outcome first = run_once(scenario, seed);
    if (!first.valid) {
      ++s.rejected;
      continue;
    }
    // Validity condition three: the identical run must produce the identical
    // event log. A run that does not repeat is not a measurement.
    const Outcome again = run_once(scenario, seed);
    if (again.digest != first.digest || again.convergence != first.convergence) {
      s.reproducible = false;
      ++s.rejected;
      continue;
    }
    ++s.valid;
    times.push_back(first.convergence);
    s.mean_update_messages += static_cast<double>(first.update_messages);
    s.mean_update_bytes += static_cast<double>(first.update_bytes);
    s.mean_route_changes += static_cast<double>(first.route_changes);
    s.mean_loops += static_cast<double>(first.loop_episodes);
    s.mean_loop_ms += static_cast<double>(first.loop_duration) / 1000.0;
  }
  if (s.valid == 0) return s;
  const double n = s.valid;
  s.min_us = *std::min_element(times.begin(), times.end());
  s.max_us = *std::max_element(times.begin(), times.end());
  s.mean_ms = static_cast<double>(std::accumulate(times.begin(), times.end(), Micros{0})) /
              n / 1000.0;
  s.mean_update_messages /= n;
  s.mean_update_bytes /= n;
  s.mean_route_changes /= n;
  s.mean_loops /= n;
  s.mean_loop_ms /= n;
  return s;
}

std::ostringstream g_csv;

void csv_row(const std::string& experiment, const std::string& label,
             const Scenario& scenario, const Summary& s) {
  g_csv << experiment << "," << label << "," << scenario.topology.nodes.size() << ","
        << scenario.topology.links.size() << "," << scenario.topology.diameter() << ","
        << scenario.topology.average_degree() << "," << scenario.hello / 1000 << ","
        << scenario.dead / 1000 << "," << s.valid << "," << s.rejected << ","
        << s.mean_ms << "," << static_cast<double>(s.min_us) / 1000.0 << ","
        << static_cast<double>(s.max_us) / 1000.0 << "," << s.mean_update_messages << ","
        << s.mean_update_bytes << "," << s.mean_route_changes << "," << s.mean_loops << ","
        << s.mean_loop_ms << "\n";
}

void heading(const char* title) {
  std::printf("\n%s\n", title);
  for (std::size_t i = 0; i < std::strlen(title); ++i) std::printf("=");
  std::printf("\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string csv_path;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
      csv_path = argv[++i];
    } else {
      std::fprintf(stderr, "usage: waypoint-bench [--csv FILE]\n");
      return 2;
    }
  }

  g_csv << "experiment,case,nodes,links,diameter,avg_degree,hello_ms,dead_ms,valid,"
           "rejected,conv_mean_ms,conv_min_ms,conv_max_ms,update_msgs,update_bytes,"
           "route_changes,loop_episodes,loop_ms\n";

  std::printf("Waypoint measurements\n");
  std::printf("%d repetitions per case, each repeated once more to confirm the "
              "event log is identical.\n",
              kRepetitions);
  std::printf("Warm up %.0f s, observation window %.0f s, all of virtual time.\n",
              static_cast<double>(kWarmUp) / 1e6, static_cast<double>(kObserve) / 1e6);

  // ---------------------------------------------------------------- experiment 1
  heading("1. Convergence against network size (ring, one link fails)");
  std::printf("%-10s %-6s %-9s %-11s %-11s %-11s %-6s\n", "topology", "nodes",
              "diameter", "mean", "min", "max", "valid");
  for (const std::size_t n : {4u, 6u, 8u, 12u, 16u, 24u, 32u}) {
    Scenario s;
    s.topology = topo::ring(n);
    s.a = 1;
    s.b = 2;
    s.detect_loops = false;  // size sweep, not a loop experiment
    const Summary r = repeat(s);
    std::printf("%-10s %-6zu %-9zu %-11.1f %-11.1f %-11.1f %d/%d%s\n",
                s.topology.name.c_str(), s.topology.nodes.size(), s.topology.diameter(),
                r.mean_ms, static_cast<double>(r.min_us) / 1000.0,
                static_cast<double>(r.max_us) / 1000.0, r.valid,
                r.valid + r.rejected, r.reproducible ? "" : "  NOT REPRODUCIBLE");
    csv_row("size", s.topology.name, s, r);
  }
  std::printf("times in milliseconds of virtual time, measured from the failure\n");

  // ---------------------------------------------------------------- experiment 2
  heading("2. Convergence against the hello and dead intervals (ring of 12)");
  std::printf("%-9s %-9s %-11s %-11s %-11s %-6s\n", "hello", "dead", "mean", "min",
              "max", "valid");
  for (const Micros hello : {250 * kMs, 500 * kMs, 1 * kSec, 2 * kSec, 4 * kSec}) {
    Scenario s;
    s.topology = topo::ring(12);
    s.hello = hello;
    s.dead = 4 * hello;  // the ratio RFC 2328 recommends
    s.a = 1;
    s.b = 2;
    s.detect_loops = false;
    const Summary r = repeat(s);
    std::printf("%-9.3f %-9.3f %-11.1f %-11.1f %-11.1f %d/%d\n",
                static_cast<double>(hello) / 1e6, static_cast<double>(s.dead) / 1e6,
                r.mean_ms, static_cast<double>(r.min_us) / 1000.0,
                static_cast<double>(r.max_us) / 1000.0, r.valid, r.valid + r.rejected);
    csv_row("timers", std::to_string(hello / 1000) + "ms", s, r);
  }
  std::printf("hello and dead in seconds, convergence in milliseconds\n");

  // ---------------------------------------------------------------- experiment 3
  heading("3. Flooding overhead against topology density (12 nodes throughout)");
  std::printf("%-10s %-6s %-8s %-11s %-12s %-11s %-11s\n", "topology", "links",
              "degree", "updates", "update bytes", "churn", "conv mean");
  {
    struct Case {
      Topology topology;
      NodeId a;
      NodeId b;
    };
    const std::vector<Case> cases = {
        {topo::ring(12), 1, 2},
        {topo::grid_mesh(3, 4), 6, 7},
        {topo::grid_mesh(2, 6), 3, 4},
        {topo::full_mesh(12), 1, 2},
    };
    for (const Case& c : cases) {
      Scenario s;
      s.topology = c.topology;
      s.a = c.a;
      s.b = c.b;
      s.detect_loops = false;
      const Summary r = repeat(s);
      std::printf("%-10s %-6zu %-8.2f %-11.1f %-12.0f %-11.1f %-11.1f\n",
                  s.topology.name.c_str(), s.topology.links.size(),
                  s.topology.average_degree(), r.mean_update_messages,
                  r.mean_update_bytes, r.mean_route_changes, r.mean_ms);
      csv_row("density", s.topology.name, s, r);
    }
  }
  std::printf("counts are means over the repetitions, from the failure to the end "
              "of the window\n");

  // ---------------------------------------------------------------- experiment 4
  heading("4. Transient loops against failure type");
  std::printf("routers install a recomputed table 0 to %.0f ms after computing it,\n"
              "drawn per router from the seed. With no spread every table changes on\n"
              "the same tick and no loop is possible by construction.\n\n",
              static_cast<double>(kInstallSpread) / 1000.0);
  std::printf("%-12s %-22s %-11s %-13s %-11s %-6s\n", "topology", "failure", "loops",
              "loop time", "conv mean", "valid");
  {
    struct Case {
      const char* label;
      Failure kind;
      NodeId a;
      NodeId b;
    };
    const Case cases[] = {
        {"link, both ways", Failure::LinkBothWays, 6, 7},
        {"link, one way only", Failure::LinkOneWay, 6, 7},
        {"node", Failure::Node, 6, 0},
    };
    const std::vector<Topology> shapes = {topo::ring(12), topo::grid_mesh(4, 4),
                                          topo::fat_tree(4)};
    for (const Topology& shape : shapes) {
      for (const Case& c : cases) {
        Scenario s;
        s.topology = shape;
        s.failure = c.kind;
        s.a = c.a;
        s.b = c.b;
        s.detect_loops = true;
        s.install_spread = kInstallSpread;
        const Summary r = repeat(s);
        std::printf("%-12s %-22s %-11.2f %-13.1f %-11.1f %d/%d\n", shape.name.c_str(),
                    c.label, r.mean_loops, r.mean_loop_ms, r.mean_ms, r.valid,
                    r.valid + r.rejected);
        csv_row("failure-type", shape.name + "/" + c.label, s, r);
      }
    }
  }
  std::printf("loops are the mean number of intervals in which some pair of nodes\n"
              "forwarded in a cycle; loop time is their total duration in ms\n");

  if (!csv_path.empty()) {
    std::ofstream out(csv_path);
    if (!out) {
      std::fprintf(stderr, "cannot write %s\n", csv_path.c_str());
      return 1;
    }
    out << g_csv.str();
    std::printf("\nwrote %s\n", csv_path.c_str());
  }
  return 0;
}
