// Wire format. Every integer is encoded little endian by hand, so the bytes on
// the wire do not depend on the endianness or the struct padding of the host.
// The same bytes travel through the simulator and through a UDP datagram.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "waypoint/types.hpp"

namespace waypoint {

using Bytes = std::vector<std::uint8_t>;
using ByteView = std::span<const std::uint8_t>;

inline constexpr std::uint8_t kProtocolVersion = 1;

enum class PacketType : std::uint8_t {
  Hello = 1,
  DbDescription = 2,
  LsRequest = 3,
  LsUpdate = 4,
  LsAck = 5,
};

// Common 8 byte header: version, type, total length, sender router id.
inline constexpr std::size_t kHeaderSize = 8;

struct Header {
  std::uint8_t version = kProtocolVersion;
  PacketType type = PacketType::Hello;
  std::uint16_t length = 0;
  NodeId sender = kNoNode;
};

struct Hello {
  std::uint32_t hello_interval_ms = 0;
  std::uint32_t dead_interval_ms = 0;
  // Routers this sender has heard from recently. Seeing our own id in the list
  // is what turns a one way hearing into a two way adjacency.
  std::vector<NodeId> seen;
};

struct Lsa {
  NodeId origin = kNoNode;
  std::uint32_t seq = 0;
  std::uint16_t age_sec = 0;
  AdjacencyList links;

  friend bool operator==(const Lsa&, const Lsa&) = default;
};

// (origin, seq) pair used by database description and acknowledgement packets.
struct LsaHandle {
  NodeId origin = kNoNode;
  std::uint32_t seq = 0;

  friend bool operator==(const LsaHandle&, const LsaHandle&) = default;
};

struct DbDescription {
  // True when this summary was sent in answer to another. Exactly one answer
  // is sent per unsolicited summary, which is what stops two routers from
  // describing their databases to each other for ever.
  bool reply = false;
  std::vector<LsaHandle> summary;
};

struct LsRequest {
  std::vector<NodeId> origins;
};

struct LsUpdate {
  std::vector<Lsa> lsas;
};

struct LsAck {
  std::vector<LsaHandle> acked;
};

// Encoding. Each function returns a complete datagram including the header.
Bytes encode_hello(NodeId sender, const Hello&);
Bytes encode_db_description(NodeId sender, const DbDescription&);
Bytes encode_ls_request(NodeId sender, const LsRequest&);
Bytes encode_ls_update(NodeId sender, const LsUpdate&);
Bytes encode_ls_ack(NodeId sender, const LsAck&);

// Decoding. Every decode validates the declared length against the buffer and
// returns nullopt on any inconsistency, so a truncated or hostile datagram
// cannot walk off the end of the buffer.
std::optional<Header> decode_header(ByteView);
std::optional<Hello> decode_hello(ByteView);
std::optional<DbDescription> decode_db_description(ByteView);
std::optional<LsRequest> decode_ls_request(ByteView);
std::optional<LsUpdate> decode_ls_update(ByteView);
std::optional<LsAck> decode_ls_ack(ByteView);

// Serialized size of one LSA, used by the flooding overhead measurement.
std::size_t lsa_wire_size(const Lsa&);

// RFC 2328 orders sequence numbers on a finite ring. `a` is newer than `b`
// when the signed difference is positive, which keeps the comparison correct
// across the wrap from 0xFFFFFFFF back to zero.
inline bool seq_newer(std::uint32_t a, std::uint32_t b) {
  return static_cast<std::int32_t>(a - b) > 0;
}

}  // namespace waypoint
