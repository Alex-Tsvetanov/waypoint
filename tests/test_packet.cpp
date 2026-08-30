#include "check.hpp"
#include "waypoint/packet.hpp"

#include <cstring>

using namespace waypoint;

namespace {

// Convert a contiguous hex string into bytes. Used to embed golden OSPFv2
// packets that were produced by an independent Python encoder and checked
// there for IP and Fletcher checksum self-consistency before being pasted in.
Bytes from_hex(const char* hex) {
  Bytes out;
  const std::size_t n = std::strlen(hex);
  CHECK_TRUE(n % 2 == 0);
  out.reserve(n / 2);
  auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  for (std::size_t i = 0; i < n; i += 2) {
    const int hi = nibble(hex[i]);
    const int lo = nibble(hex[i + 1]);
    CHECK_TRUE(hi >= 0 && lo >= 0);
    out.push_back(static_cast<std::uint8_t>((hi << 4) | lo));
  }
  return out;
}

// Independent IP checksum (auth excluded) so a test can mutate a body field
// and still present a packet whose only fault is the mutated field.
void repair_packet_checksum(Bytes& packet) {
  CHECK_TRUE(packet.size() >= kHeaderSize);
  packet[12] = 0;
  packet[13] = 0;
  std::uint32_t sum = 0;
  auto add = [&](std::uint8_t hi, std::uint8_t lo) {
    sum += static_cast<std::uint32_t>((static_cast<std::uint16_t>(hi) << 8) | lo);
  };
  for (std::size_t i = 0; i + 1 < 12; i += 2) add(packet[i], packet[i + 1]);
  add(0, 0);
  add(packet[14], packet[15]);
  for (std::size_t i = kHeaderSize; i + 1 < packet.size(); i += 2) {
    add(packet[i], packet[i + 1]);
  }
  if ((packet.size() - kHeaderSize) % 2 == 1) add(packet.back(), 0);
  while (sum >> 16) sum = (sum & 0xFFFFu) + (sum >> 16);
  const std::uint16_t csum = static_cast<std::uint16_t>(~sum);
  packet[12] = static_cast<std::uint8_t>((csum >> 8) & 0xFF);
  packet[13] = static_cast<std::uint8_t>(csum & 0xFF);
}

}  // namespace

WP_TEST(packet, hello_round_trip) {
  Hello h;
  h.hello_interval_ms = 1000;
  h.dead_interval_ms = 4000;
  h.seen = {7, 9, 4000000000u};

  const Bytes wire = encode_hello(42, h);
  const auto header = decode_header(wire);
  CHECK_TRUE(header.has_value());
  CHECK_EQ(header->sender, 42u);
  CHECK_TRUE(header->type == PacketType::Hello);
  CHECK_EQ(static_cast<std::size_t>(header->length), wire.size());
  CHECK_EQ(header->version, kProtocolVersion);
  CHECK_EQ(header->area, kOspfAreaId);

  const auto back = decode_hello(wire);
  CHECK_TRUE(back.has_value());
  CHECK_EQ(back->hello_interval_ms, 1000u);
  CHECK_EQ(back->dead_interval_ms, 4000u);
  CHECK_TRUE(back->seen == h.seen);
}

WP_TEST(packet, ls_update_round_trip) {
  LsUpdate u;
  Lsa a;
  a.origin = 3;
  a.seq = 0x80000001u;
  a.age_sec = 17;
  a.links = {{1, 10}, {2, 55}};
  Lsa b;
  b.origin = 4;
  b.seq = 1;
  b.links = {};
  u.lsas = {a, b};

  const Bytes wire = encode_ls_update(3, u);
  const auto back = decode_ls_update(wire);
  CHECK_TRUE(back.has_value());
  CHECK_EQ(back->lsas.size(), std::size_t{2});
  CHECK_TRUE(back->lsas[0] == a);
  CHECK_TRUE(back->lsas[1] == b);
}

WP_TEST(packet, db_description_and_ack_round_trip) {
  DbDescription d;
  d.reply = true;
  d.summary = {{1, 5}, {2, 9}};
  const auto back = decode_db_description(encode_db_description(1, d));
  CHECK_TRUE(back.has_value());
  CHECK_TRUE(back->summary == d.summary);
  CHECK_TRUE(back->reply);
  d.reply = false;
  CHECK_FALSE(decode_db_description(encode_db_description(1, d))->reply);

  LsAck ack;
  ack.acked = {{3, 11}};
  const auto ack_back = decode_ls_ack(encode_ls_ack(2, ack));
  CHECK_TRUE(ack_back.has_value());
  CHECK_TRUE(ack_back->acked == ack.acked);

  LsRequest req;
  req.origins = {8, 9, 10};
  const auto req_back = decode_ls_request(encode_ls_request(2, req));
  CHECK_TRUE(req_back.has_value());
  CHECK_TRUE(req_back->origins == req.origins);
}

WP_TEST(packet, truncated_input_is_rejected) {
  Hello h;
  h.hello_interval_ms = 1000;
  h.dead_interval_ms = 4000;
  h.seen = {1, 2, 3};
  Bytes wire = encode_hello(1, h);

  for (std::size_t cut = 1; cut < wire.size(); ++cut) {
    Bytes shorter(wire.begin(), wire.begin() + static_cast<std::ptrdiff_t>(cut));
    CHECK_FALSE(decode_hello(shorter).has_value());
  }
  CHECK_FALSE(decode_header(Bytes{}).has_value());
}

WP_TEST(packet, wrong_type_and_version_rejected) {
  Bytes wire = encode_hello(1, Hello{});
  // A hello decoded as an update must fail rather than reinterpret the bytes.
  CHECK_FALSE(decode_ls_update(wire).has_value());

  wire[0] = 99;  // version
  CHECK_FALSE(decode_header(wire).has_value());

  Bytes other = encode_hello(1, Hello{});
  other[1] = 42;  // unknown packet type
  // Length still matches, but type is illegal; checksum is also wrong once
  // the type byte changes, so either check may reject — both must reject.
  CHECK_FALSE(decode_header(other).has_value());
}

WP_TEST(packet, absurd_lsa_count_is_rejected) {
  // The LSA count field in an LS Update is attacker controlled. A count far
  // beyond what the datagram can hold must be refused before any allocation.
  Bytes wire = encode_ls_update(1, LsUpdate{});
  wire[kHeaderSize + 0] = 0xFF;
  wire[kHeaderSize + 1] = 0xFF;
  wire[kHeaderSize + 2] = 0xFF;
  wire[kHeaderSize + 3] = 0xFF;
  repair_packet_checksum(wire);
  CHECK_TRUE(decode_header(wire).has_value());  // framing is fine
  CHECK_FALSE(decode_ls_update(wire).has_value());
}

WP_TEST(packet, trailing_garbage_is_rejected) {
  Bytes wire = encode_hello(1, Hello{1000, 4000, {}});
  wire.push_back(0);
  // The length field no longer matches the buffer.
  CHECK_FALSE(decode_hello(wire).has_value());
}

WP_TEST(packet, bad_packet_checksum_is_rejected) {
  Bytes wire = encode_hello(1, Hello{1000, 4000, {7}});
  wire[12] ^= 0xFF;  // flip a checksum byte
  CHECK_FALSE(decode_header(wire).has_value());
}

WP_TEST(packet, bad_lsa_checksum_is_rejected) {
  Lsa a;
  a.origin = 3;
  a.seq = 0x80000001u;
  a.links = {{1, 10}};
  Bytes wire = encode_ls_update(3, LsUpdate{{a}});
  // Fletcher checksum sits at LSA offset 16 within the update body
  // (header 24 + count 4 + age/options/type/ids/seq = offset 44).
  const std::size_t csum_off = kHeaderSize + 4 + 16;
  wire[csum_off] ^= 0xFF;
  repair_packet_checksum(wire);
  CHECK_TRUE(decode_header(wire).has_value());
  CHECK_FALSE(decode_ls_update(wire).has_value());
}

WP_TEST(packet, sequence_numbers_compare_across_the_wrap) {
  CHECK_TRUE(seq_newer(2, 1));
  CHECK_FALSE(seq_newer(1, 2));
  CHECK_FALSE(seq_newer(5, 5));
  // 0 is newer than 0xFFFFFFFF: the ring wrapped, it did not restart.
  CHECK_TRUE(seq_newer(0, 0xFFFFFFFFu));
  CHECK_FALSE(seq_newer(0xFFFFFFFFu, 0));
}

WP_TEST(packet, lsa_wire_size_matches_encoding) {
  Lsa a;
  a.origin = 1;
  a.links = {{2, 3}, {4, 5}, {6, 7}};
  const Bytes wire = encode_ls_update(1, LsUpdate{{a}});
  CHECK_EQ(wire.size(), kHeaderSize + 4 + lsa_wire_size(a));
}

// --- Golden / fixture packets ------------------------------------------------
// The hex strings below were generated by an independent Python encoder that
// implements the same RFC 2328 layout, IP checksum (auth field excluded) and
// Fletcher LSA checksum, then self-checked. They are not taken from the C++
// encoder under test.

WP_TEST(packet, golden_hello_matches_rfc2328_layout) {
  // Router 192.0.2.1, area 0, HelloInterval 10 s, DeadInterval 40 s,
  // one neighbour 192.0.2.2, null auth, Options E, Priority 0.
  const Bytes expected = from_hex(
      "02010030c000020100000000779800000000000000000000"
      "00000000000a0200000000280000000000000000c0000202");
  Hello h;
  h.hello_interval_ms = 10000;
  h.dead_interval_ms = 40000;
  h.seen = {0xC0000202u};
  const Bytes wire = encode_hello(0xC0000201u, h);
  CHECK_EQ(wire.size(), expected.size());
  CHECK_TRUE(wire == expected);

  const auto back = decode_hello(expected);
  CHECK_TRUE(back.has_value());
  CHECK_EQ(back->hello_interval_ms, 10000u);
  CHECK_EQ(back->dead_interval_ms, 40000u);
  CHECK_EQ(back->seen.size(), std::size_t{1});
  CHECK_EQ(back->seen[0], 0xC0000202u);
}

WP_TEST(packet, golden_ls_update_router_lsa) {
  // One Router-LSA from 192.0.2.1, seq 0x80000001, one p2p link to
  // 192.0.2.2 metric 10. Fletcher checksum 0x08f6 verified independently.
  const Bytes expected = from_hex(
      "02040040c000020100000000698b00000000000000000000"
      "00000001"
      "00000201c0000201c00002018000000108f6002400000001"
      "c0000202000000000100000a");
  Lsa lsa;
  lsa.origin = 0xC0000201u;
  lsa.seq = 0x80000001u;
  lsa.age_sec = 0;
  lsa.links = {{0xC0000202u, 10}};
  const Bytes wire = encode_ls_update(0xC0000201u, LsUpdate{{lsa}});
  CHECK_TRUE(wire == expected);

  const auto back = decode_ls_update(expected);
  CHECK_TRUE(back.has_value());
  CHECK_EQ(back->lsas.size(), std::size_t{1});
  CHECK_TRUE(back->lsas[0] == lsa);
}

WP_TEST(packet, golden_db_description_and_ack_and_request) {
  const Bytes dd_expected = from_hex(
      "02020034c0000201000000007b9e00000000000000000000"
      "05dc020500000001"
      "00000201c0000201c000020180000001b22d0014");
  DbDescription d;
  d.reply = false;
  d.summary = {{0xC0000201u, 0x80000001u}};
  CHECK_TRUE(encode_db_description(0xC0000201u, d) == dd_expected);
  const auto dd_back = decode_db_description(dd_expected);
  CHECK_TRUE(dd_back.has_value());
  CHECK_FALSE(dd_back->reply);
  CHECK_TRUE(dd_back->summary == d.summary);

  const Bytes ack_expected = from_hex(
      "0205002cc000020100000000838500000000000000000000"
      "00000201c0000201c000020180000001b22d0014");
  LsAck ack;
  ack.acked = {{0xC0000201u, 0x80000001u}};
  CHECK_TRUE(encode_ls_ack(0xC0000201u, ack) == ack_expected);

  const Bytes req_expected = from_hex(
      "02030024c000020100000000b7d200000000000000000000"
      "00000001c0000201c0000201");
  LsRequest req;
  req.origins = {0xC0000201u};
  CHECK_TRUE(encode_ls_request(0xC0000201u, req) == req_expected);
  const auto req_back = decode_ls_request(req_expected);
  CHECK_TRUE(req_back.has_value());
  CHECK_TRUE(req_back->origins == req.origins);
}

WP_TEST(packet, header_is_twenty_four_bytes_network_order) {
  const Bytes wire = encode_hello(0x01020304u, Hello{1000, 4000, {}});
  CHECK_TRUE(wire.size() >= kHeaderSize);
  CHECK_EQ(wire[0], std::uint8_t{2});             // version
  CHECK_EQ(wire[1], std::uint8_t{1});             // Hello
  CHECK_EQ(wire[4], std::uint8_t{0x01});          // Router ID big-endian
  CHECK_EQ(wire[5], std::uint8_t{0x02});
  CHECK_EQ(wire[6], std::uint8_t{0x03});
  CHECK_EQ(wire[7], std::uint8_t{0x04});
  CHECK_EQ(wire[8], std::uint8_t{0});             // Area ID 0.0.0.0
  CHECK_EQ(wire[9], std::uint8_t{0});
  CHECK_EQ(wire[10], std::uint8_t{0});
  CHECK_EQ(wire[11], std::uint8_t{0});
  CHECK_EQ(wire[14], std::uint8_t{0});            // AuType null
  CHECK_EQ(wire[15], std::uint8_t{0});
}
