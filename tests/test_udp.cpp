// Live mode over real sockets. These cases exist to prove that the protocol
// core is genuinely transport independent: not one line of Router changes
// between the simulator and a pair of UDP sockets on the loopback interface.
#include <memory>

#include "check.hpp"
#include "waypoint/udp.hpp"

using namespace waypoint;

namespace {

Router::Config live_config(NodeId id) {
  Router::Config c;
  c.id = id;
  c.hello_interval = 50 * kMs;
  c.dead_interval = 250 * kMs;
  c.rxmt_interval = 50 * kMs;
  c.spf_delay = 10 * kMs;
  return c;
}

}  // namespace

WP_TEST(udp, a_socket_binds_and_reports_its_port) {
  UdpSocket socket;
  std::string error;
  CHECK_TRUE(socket.open(0, error));
  CHECK_TRUE(error.empty());
  CHECK_TRUE(socket.is_open());
  CHECK_TRUE(socket.local_port() != 0);
  socket.close();
  CHECK_FALSE(socket.is_open());
}

WP_TEST(udp, a_datagram_survives_the_loopback_unchanged) {
  UdpSocket socket;
  std::string error;
  CHECK_TRUE(socket.open(0, error));

  const Bytes wire = encode_hello(7, Hello{100, 400, {1, 2, 3}});
  CHECK_TRUE(socket.send_to(Endpoint{"127.0.0.1", socket.local_port()}, wire, error));

  Bytes received;
  CHECK_TRUE(socket.receive(received, 2 * kSec) == UdpSocket::Receive::Datagram);
  CHECK_TRUE(received == wire);

  const auto decoded = decode_hello(received);
  CHECK_TRUE(decoded.has_value());
  CHECK_EQ(decoded->dead_interval_ms, 400u);
}

WP_TEST(udp, receive_reports_a_timeout_rather_than_blocking) {
  UdpSocket socket;
  std::string error;
  CHECK_TRUE(socket.open(0, error));
  Bytes buffer;
  CHECK_TRUE(socket.receive(buffer, 20 * kMs) == UdpSocket::Receive::TimedOut);
}

WP_TEST(udp, sending_to_a_name_that_is_not_an_address_fails_cleanly) {
  UdpSocket socket;
  std::string error;
  CHECK_TRUE(socket.open(0, error));
  CHECK_FALSE(socket.send_to(Endpoint{"not-an-address", 9}, Bytes{1, 2}, error));
  CHECK_FALSE(error.empty());
}

WP_TEST(udp, two_routers_form_an_adjacency_over_real_sockets) {
  UdpSocket socket_a;
  UdpSocket socket_b;
  std::string error;
  CHECK_TRUE(socket_a.open(0, error));
  CHECK_TRUE(socket_b.open(0, error));

  const Endpoint at_a{"127.0.0.1", socket_a.local_port()};
  const Endpoint at_b{"127.0.0.1", socket_b.local_port()};

  UdpEnv env_a({{2, at_b}}, socket_a);
  UdpEnv env_b({{1, at_a}}, socket_b);

  Router router_a(live_config(1), env_a);
  Router router_b(live_config(2), env_b);
  router_a.add_link(2, 10);
  router_b.add_link(1, 10);
  router_a.start();
  router_b.start();

  // Alternate short slices so both event loops make progress on one thread.
  // Generous budget: the assertion is that the adjacency forms at all, not
  // that it forms within a particular number of milliseconds.
  bool ready = false;
  for (int slice = 0; slice < 400 && !ready; ++slice) {
    env_a.pump(router_a, 10 * kMs);
    env_b.pump(router_b, 10 * kMs);
    ready = router_a.state_of(2) == NeighborState::Full &&
            router_b.state_of(1) == NeighborState::Full &&
            router_a.rib().size() == 1 && router_b.rib().size() == 1;
  }

  CHECK_TRUE(ready);
  CHECK_TRUE(router_a.state_of(2) == NeighborState::Full);
  CHECK_TRUE(router_b.state_of(1) == NeighborState::Full);
  CHECK_EQ(router_a.rib().at(2).cost, 10u);
  CHECK_EQ(router_b.rib().at(1).cost, 10u);
  CHECK_EQ(router_a.rib().at(2).next_hops.front(), 2u);
  CHECK_FALSE(router_a.malformed_seen());
  CHECK_FALSE(router_b.malformed_seen());
  CHECK_TRUE(env_a.datagrams_sent() > 0);
  CHECK_TRUE(env_b.datagrams_received() > 0);
  CHECK_EQ(env_a.send_failures(), std::uint64_t{0});
}

WP_TEST(udp, a_datagram_from_a_stranger_is_discarded) {
  UdpSocket socket_a;
  UdpSocket intruder;
  std::string error;
  CHECK_TRUE(socket_a.open(0, error));
  CHECK_TRUE(intruder.open(0, error));

  UdpEnv env_a({{2, Endpoint{"127.0.0.1", intruder.local_port()}}}, socket_a);
  Router router_a(live_config(1), env_a);
  router_a.add_link(2, 10);
  router_a.start();

  // Router 99 is not a configured interface of router 1.
  const Bytes hostile = encode_ls_update(99, LsUpdate{{Lsa{99, 1, 0, {{1, 1}}}}});
  CHECK_TRUE(intruder.send_to(Endpoint{"127.0.0.1", socket_a.local_port()}, hostile,
                              error));
  env_a.pump(router_a, 100 * kMs);

  CHECK_TRUE(router_a.lsdb().find(99) == nullptr);
  CHECK_TRUE(router_a.malformed_count() > 0);
}
