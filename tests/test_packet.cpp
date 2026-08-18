#include "check.hpp"
#include "waypoint/packet.hpp"

using namespace waypoint;

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
  CHECK_FALSE(decode_header(other).has_value());
}

WP_TEST(packet, absurd_element_count_is_rejected) {
  // The count field is attacker controlled. A count far beyond what the
  // datagram can hold must be refused before any allocation happens.
  Bytes wire = encode_hello(1, Hello{});
  wire[kHeaderSize + 8] = 0xFF;
  wire[kHeaderSize + 9] = 0xFF;
  wire[kHeaderSize + 10] = 0xFF;
  wire[kHeaderSize + 11] = 0xFF;
  CHECK_FALSE(decode_hello(wire).has_value());
}

WP_TEST(packet, trailing_garbage_is_rejected) {
  Bytes wire = encode_hello(1, Hello{});
  wire.push_back(0);
  // The length field no longer matches the buffer.
  CHECK_FALSE(decode_hello(wire).has_value());
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
