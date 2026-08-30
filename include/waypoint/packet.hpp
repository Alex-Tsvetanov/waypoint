// OSPFv2 wire format (RFC 2328). Every multi-byte integer is encoded in
// network byte order by hand, so the bytes on the wire do not depend on the
// endianness or the struct padding of the host. The same bytes travel through
// the simulator and through a UDP datagram. The logical packet types match the
// five OSPFv2 types this daemon uses; fields the protocol core does not use
// (area, authentication, DR/BDR, DD sequence) are filled with RFC-conformant
// defaults so an external parser can read the packet.
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

// OSPFv2 version number (RFC 2328 Appendix A.3.1).
inline constexpr std::uint8_t kProtocolVersion = 2;

// Backbone area 0.0.0.0. The daemon does not partition areas; every packet
// carries this value so the header matches what FRRouting/BIRD expect.
inline constexpr std::uint32_t kOspfAreaId = 0;

// Null authentication (AuType 0). The 64-bit authentication field is zero.
inline constexpr std::uint16_t kOspfAuTypeNull = 0;

// Options bit E: external routing capability. Set on Hellos, DD and LSAs.
inline constexpr std::uint8_t kOspfOptionE = 0x02;

// Router-LSA (LS type 1) and point-to-point link type inside a Router-LSA.
inline constexpr std::uint8_t kOspfLsTypeRouter = 1;
inline constexpr std::uint8_t kOspfLinkTypeP2P = 1;

// Database Description flags (RFC 2328 Appendix A.3.3).
inline constexpr std::uint8_t kOspfDdFlagMs = 0x01;
inline constexpr std::uint8_t kOspfDdFlagM = 0x02;
inline constexpr std::uint8_t kOspfDdFlagI = 0x04;

enum class PacketType : std::uint8_t {
  Hello = 1,
  DbDescription = 2,
  LsRequest = 3,
  LsUpdate = 4,
  LsAck = 5,
};

// OSPFv2 common header is 24 bytes (RFC 2328 Appendix A.3.1).
inline constexpr std::size_t kHeaderSize = 24;

// Router-LSA / LSA header size on the wire.
inline constexpr std::size_t kLsaHeaderSize = 20;

struct Header {
  std::uint8_t version = kProtocolVersion;
  PacketType type = PacketType::Hello;
  std::uint16_t length = 0;
  NodeId sender = kNoNode;
  std::uint32_t area = kOspfAreaId;
  std::uint16_t checksum = 0;
  std::uint16_t au_type = kOspfAuTypeNull;
};

struct Hello {
  // Intervals are kept in milliseconds at the API boundary (the rest of the
  // daemon works in Micros). On the wire OSPFv2 carries whole seconds.
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
// On the wire this is an OSPFv2 LSA header with LS type Router-LSA and both
// Link State ID and Advertising Router equal to origin.
struct LsaHandle {
  NodeId origin = kNoNode;
  std::uint32_t seq = 0;

  friend bool operator==(const LsaHandle&, const LsaHandle&) = default;
};

struct DbDescription {
  // True when this summary was sent in answer to another. Encoded as the
  // Master/Slave bit of the DD flags: unsolicited ⇒ Master (MS=1, I=1);
  // reply ⇒ Slave (MS=0, I=0).
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

// Encoding. Each function returns a complete OSPFv2 datagram including the
// 24-byte header and a correct IP checksum (authentication excluded).
Bytes encode_hello(NodeId sender, const Hello&);
Bytes encode_db_description(NodeId sender, const DbDescription&);
Bytes encode_ls_request(NodeId sender, const LsRequest&);
Bytes encode_ls_update(NodeId sender, const LsUpdate&);
Bytes encode_ls_ack(NodeId sender, const LsAck&);

// Decoding. Every decode validates the declared length, the packet checksum
// and, for LSAs, the Fletcher checksum. Returns nullopt on any inconsistency.
std::optional<Header> decode_header(ByteView);
std::optional<Hello> decode_hello(ByteView);
std::optional<DbDescription> decode_db_description(ByteView);
std::optional<LsRequest> decode_ls_request(ByteView);
std::optional<LsUpdate> decode_ls_update(ByteView);
std::optional<LsAck> decode_ls_ack(ByteView);

// Serialized size of one Router-LSA on the wire (header + body), used by the
// flooding overhead measurement.
std::size_t lsa_wire_size(const Lsa&);

// RFC 2328 orders sequence numbers on a finite ring. `a` is newer than `b`
// when the signed difference is positive, which keeps the comparison correct
// across the wrap from 0xFFFFFFFF back to zero.
inline bool seq_newer(std::uint32_t a, std::uint32_t b) {
  return static_cast<std::int32_t>(a - b) > 0;
}

}  // namespace waypoint
