#include "waypoint/packet.hpp"

namespace waypoint {
namespace {

void put_u8(Bytes& out, std::uint8_t v) { out.push_back(v); }

void put_u16(Bytes& out, std::uint16_t v) {
  out.push_back(static_cast<std::uint8_t>(v & 0xFF));
  out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
}

void put_u32(Bytes& out, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF));
  }
}

// Bounds checked cursor over an incoming datagram. Every read either advances
// the cursor or marks the reader bad; the bad flag is checked once at the end
// rather than at every field.
class Reader {
 public:
  explicit Reader(ByteView data) : data_(data) {}

  std::uint8_t u8() {
    if (pos_ + 1 > data_.size()) { bad_ = true; return 0; }
    return data_[pos_++];
  }
  std::uint16_t u16() {
    if (pos_ + 2 > data_.size()) { bad_ = true; return 0; }
    std::uint16_t v = static_cast<std::uint16_t>(
        data_[pos_] | (static_cast<std::uint16_t>(data_[pos_ + 1]) << 8));
    pos_ += 2;
    return v;
  }
  std::uint32_t u32() {
    if (pos_ + 4 > data_.size()) { bad_ = true; return 0; }
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
      v |= static_cast<std::uint32_t>(data_[pos_ + static_cast<std::size_t>(i)]) << (8 * i);
    }
    pos_ += 4;
    return v;
  }

  bool bad() const { return bad_; }
  std::size_t remaining() const { return bad_ ? 0 : data_.size() - pos_; }

 private:
  ByteView data_;
  std::size_t pos_ = 0;
  bool bad_ = false;
};

// A declared element count is attacker controlled, so it is rejected up front
// when the buffer cannot possibly hold that many elements. Without this a
// count of 2^32-1 would make the vector reserve before the reader ever fails.
bool count_fits(const Reader& r, std::uint32_t count, std::size_t element_size) {
  return static_cast<std::uint64_t>(count) * element_size <= r.remaining();
}

Bytes begin_packet(PacketType type, NodeId sender) {
  Bytes out;
  put_u8(out, kProtocolVersion);
  put_u8(out, static_cast<std::uint8_t>(type));
  put_u16(out, 0);  // patched by finish_packet
  put_u32(out, sender);
  return out;
}

void finish_packet(Bytes& out) {
  const std::uint16_t len = static_cast<std::uint16_t>(out.size());
  out[2] = static_cast<std::uint8_t>(len & 0xFF);
  out[3] = static_cast<std::uint8_t>((len >> 8) & 0xFF);
}

void put_lsa(Bytes& out, const Lsa& lsa) {
  put_u32(out, lsa.origin);
  put_u32(out, lsa.seq);
  put_u16(out, lsa.age_sec);
  put_u16(out, static_cast<std::uint16_t>(lsa.links.size()));
  for (const Adjacency& a : lsa.links) {
    put_u32(out, a.peer);
    put_u32(out, a.cost);
  }
}

// Reads a packet header and rejects anything whose declared length does not
// match the buffer exactly. Returns a reader positioned after the header.
std::optional<Reader> open_body(ByteView data, PacketType expected) {
  const std::optional<Header> h = decode_header(data);
  if (!h || h->type != expected) return std::nullopt;
  Reader r(data);
  r.u8(); r.u8(); r.u16(); r.u32();
  return r;
}

}  // namespace

std::size_t lsa_wire_size(const Lsa& lsa) {
  return 12 + 8 * lsa.links.size();
}

Bytes encode_hello(NodeId sender, const Hello& h) {
  Bytes out = begin_packet(PacketType::Hello, sender);
  put_u32(out, h.hello_interval_ms);
  put_u32(out, h.dead_interval_ms);
  put_u32(out, static_cast<std::uint32_t>(h.seen.size()));
  for (NodeId n : h.seen) put_u32(out, n);
  finish_packet(out);
  return out;
}

Bytes encode_db_description(NodeId sender, const DbDescription& d) {
  Bytes out = begin_packet(PacketType::DbDescription, sender);
  put_u32(out, d.reply ? 1u : 0u);
  put_u32(out, static_cast<std::uint32_t>(d.summary.size()));
  for (const LsaHandle& s : d.summary) {
    put_u32(out, s.origin);
    put_u32(out, s.seq);
  }
  finish_packet(out);
  return out;
}

Bytes encode_ls_request(NodeId sender, const LsRequest& r) {
  Bytes out = begin_packet(PacketType::LsRequest, sender);
  put_u32(out, static_cast<std::uint32_t>(r.origins.size()));
  for (NodeId n : r.origins) put_u32(out, n);
  finish_packet(out);
  return out;
}

Bytes encode_ls_update(NodeId sender, const LsUpdate& u) {
  Bytes out = begin_packet(PacketType::LsUpdate, sender);
  put_u32(out, static_cast<std::uint32_t>(u.lsas.size()));
  for (const Lsa& l : u.lsas) put_lsa(out, l);
  finish_packet(out);
  return out;
}

Bytes encode_ls_ack(NodeId sender, const LsAck& a) {
  Bytes out = begin_packet(PacketType::LsAck, sender);
  put_u32(out, static_cast<std::uint32_t>(a.acked.size()));
  for (const LsaHandle& s : a.acked) {
    put_u32(out, s.origin);
    put_u32(out, s.seq);
  }
  finish_packet(out);
  return out;
}

std::optional<Header> decode_header(ByteView data) {
  if (data.size() < kHeaderSize) return std::nullopt;
  Reader r(data);
  Header h;
  h.version = r.u8();
  const std::uint8_t type = r.u8();
  h.length = r.u16();
  h.sender = r.u32();
  if (r.bad()) return std::nullopt;
  if (h.version != kProtocolVersion) return std::nullopt;
  if (type < 1 || type > 5) return std::nullopt;
  h.type = static_cast<PacketType>(type);
  // A length field that disagrees with the datagram is a framing error, not a
  // recoverable difference of opinion.
  if (h.length != data.size()) return std::nullopt;
  return h;
}

std::optional<Hello> decode_hello(ByteView data) {
  auto reader = open_body(data, PacketType::Hello);
  if (!reader) return std::nullopt;
  Reader& r = *reader;
  Hello h;
  h.hello_interval_ms = r.u32();
  h.dead_interval_ms = r.u32();
  const std::uint32_t count = r.u32();
  if (r.bad() || !count_fits(r, count, 4)) return std::nullopt;
  h.seen.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) h.seen.push_back(r.u32());
  if (r.bad() || r.remaining() != 0) return std::nullopt;
  return h;
}

std::optional<DbDescription> decode_db_description(ByteView data) {
  auto reader = open_body(data, PacketType::DbDescription);
  if (!reader) return std::nullopt;
  Reader& r = *reader;
  DbDescription d;
  d.reply = r.u32() != 0;
  const std::uint32_t count = r.u32();
  if (r.bad() || !count_fits(r, count, 8)) return std::nullopt;
  d.summary.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    LsaHandle s;
    s.origin = r.u32();
    s.seq = r.u32();
    d.summary.push_back(s);
  }
  if (r.bad() || r.remaining() != 0) return std::nullopt;
  return d;
}

std::optional<LsRequest> decode_ls_request(ByteView data) {
  auto reader = open_body(data, PacketType::LsRequest);
  if (!reader) return std::nullopt;
  Reader& r = *reader;
  LsRequest req;
  const std::uint32_t count = r.u32();
  if (r.bad() || !count_fits(r, count, 4)) return std::nullopt;
  req.origins.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) req.origins.push_back(r.u32());
  if (r.bad() || r.remaining() != 0) return std::nullopt;
  return req;
}

std::optional<LsUpdate> decode_ls_update(ByteView data) {
  auto reader = open_body(data, PacketType::LsUpdate);
  if (!reader) return std::nullopt;
  Reader& r = *reader;
  LsUpdate u;
  const std::uint32_t count = r.u32();
  if (r.bad() || !count_fits(r, count, 12)) return std::nullopt;
  u.lsas.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    Lsa l;
    l.origin = r.u32();
    l.seq = r.u32();
    l.age_sec = r.u16();
    const std::uint16_t links = r.u16();
    if (r.bad() || !count_fits(r, links, 8)) return std::nullopt;
    l.links.reserve(links);
    for (std::uint16_t j = 0; j < links; ++j) {
      Adjacency a;
      a.peer = r.u32();
      a.cost = r.u32();
      l.links.push_back(a);
    }
    u.lsas.push_back(std::move(l));
  }
  if (r.bad() || r.remaining() != 0) return std::nullopt;
  return u;
}

std::optional<LsAck> decode_ls_ack(ByteView data) {
  auto reader = open_body(data, PacketType::LsAck);
  if (!reader) return std::nullopt;
  Reader& r = *reader;
  LsAck a;
  const std::uint32_t count = r.u32();
  if (r.bad() || !count_fits(r, count, 8)) return std::nullopt;
  a.acked.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    LsaHandle s;
    s.origin = r.u32();
    s.seq = r.u32();
    a.acked.push_back(s);
  }
  if (r.bad() || r.remaining() != 0) return std::nullopt;
  return a;
}

}  // namespace waypoint
