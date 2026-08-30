#include "waypoint/packet.hpp"

#include <algorithm>

namespace waypoint {
namespace {

void put_u8(Bytes& out, std::uint8_t v) { out.push_back(v); }

void put_u16_be(Bytes& out, std::uint16_t v) {
  out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
  out.push_back(static_cast<std::uint8_t>(v & 0xFF));
}

void put_u32_be(Bytes& out, std::uint32_t v) {
  out.push_back(static_cast<std::uint8_t>((v >> 24) & 0xFF));
  out.push_back(static_cast<std::uint8_t>((v >> 16) & 0xFF));
  out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
  out.push_back(static_cast<std::uint8_t>(v & 0xFF));
}

class Reader {
 public:
  explicit Reader(ByteView data) : data_(data) {}

  std::uint8_t u8() {
    if (pos_ + 1 > data_.size()) {
      bad_ = true;
      return 0;
    }
    return data_[pos_++];
  }

  std::uint16_t u16() {
    if (pos_ + 2 > data_.size()) {
      bad_ = true;
      return 0;
    }
    const std::uint16_t v = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data_[pos_]) << 8) | data_[pos_ + 1]);
    pos_ += 2;
    return v;
  }

  std::uint32_t u32() {
    if (pos_ + 4 > data_.size()) {
      bad_ = true;
      return 0;
    }
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
      v = (v << 8) | data_[pos_ + static_cast<std::size_t>(i)];
    }
    pos_ += 4;
    return v;
  }

  ByteView take(std::size_t n) {
    if (pos_ + n > data_.size()) {
      bad_ = true;
      return {};
    }
    ByteView v = data_.subspan(pos_, n);
    pos_ += n;
    return v;
  }

  bool bad() const { return bad_; }
  std::size_t remaining() const { return bad_ ? 0 : data_.size() - pos_; }

 private:
  ByteView data_;
  std::size_t pos_ = 0;
  bool bad_ = false;
};

// Standard IP checksum over the OSPF packet excluding the 64-bit
// authentication field (RFC 2328 §12.1.5 / Appendix A.3.1).
std::uint16_t ospf_packet_checksum(ByteView packet) {
  if (packet.size() < kHeaderSize) return 0;
  std::uint32_t sum = 0;
  auto add_word = [&](std::uint8_t hi, std::uint8_t lo) {
    sum += static_cast<std::uint32_t>((static_cast<std::uint16_t>(hi) << 8) | lo);
  };

  // Bytes 0..11 (through Area ID), then checksum field treated as zero
  // (bytes 12..13), then AuType (14..15). Skip Authentication (16..23).
  for (std::size_t i = 0; i + 1 < 12; i += 2) add_word(packet[i], packet[i + 1]);
  add_word(0, 0);  // checksum
  add_word(packet[14], packet[15]);  // AuType

  for (std::size_t i = kHeaderSize; i + 1 < packet.size(); i += 2) {
    add_word(packet[i], packet[i + 1]);
  }
  if ((packet.size() - kHeaderSize) % 2 == 1) {
    add_word(packet[packet.size() - 1], 0);
  }

  while (sum >> 16) sum = (sum & 0xFFFFu) + (sum >> 16);
  return static_cast<std::uint16_t>(~sum);
}

// Fletcher checksum over an LSA starting at the Options field (RFC 2328
// Appendix B). `lsa` is the full LSA including the Age field; the checksum
// bytes themselves are treated as zero during the calculation.
std::uint16_t ospf_lsa_checksum(ByteView lsa) {
  if (lsa.size() < kLsaHeaderSize) return 0;
  const std::uint16_t declared =
      static_cast<std::uint16_t>((static_cast<std::uint16_t>(lsa[18]) << 8) | lsa[19]);
  if (declared < kLsaHeaderSize || declared > lsa.size()) return 0;

  // Length of the covered region: LSA length minus the two Age bytes.
  const std::size_t cover = static_cast<std::size_t>(declared) - 2;
  int c0 = 0;
  int c1 = 0;
  for (std::size_t i = 0; i < cover; ++i) {
    const std::size_t off = 2 + i;  // start at Options
    std::uint8_t byte = lsa[off];
    // Checksum field sits at LSA offsets 16..17 ⇒ cover offsets 14..15.
    if (i == 14 || i == 15) byte = 0;
    c0 += byte;
    c1 += c0;
  }
  c0 %= 255;
  c1 %= 255;

  int x = ((static_cast<int>(cover) - 15) * c0 - c1) % 255;
  if (x <= 0) x += 255;
  int y = 510 - c0 - x;
  if (y > 255) y -= 255;
  return static_cast<std::uint16_t>(((x & 0xFF) << 8) | (y & 0xFF));
}

void write_lsa_header_fields(Bytes& out, std::uint16_t age, NodeId origin,
                             std::uint32_t seq, std::uint16_t length) {
  put_u16_be(out, age);
  put_u8(out, kOspfOptionE);
  put_u8(out, kOspfLsTypeRouter);
  put_u32_be(out, origin);  // Link State ID
  put_u32_be(out, origin);  // Advertising Router
  put_u32_be(out, seq);
  put_u16_be(out, 0);  // checksum, patched below
  put_u16_be(out, length);
}

void patch_lsa_checksum(Bytes& lsa, std::size_t offset) {
  const ByteView view(lsa.data() + offset, lsa.size() - offset);
  const std::uint16_t csum = ospf_lsa_checksum(view);
  lsa[offset + 16] = static_cast<std::uint8_t>((csum >> 8) & 0xFF);
  lsa[offset + 17] = static_cast<std::uint8_t>(csum & 0xFF);
}

void put_lsa_header(Bytes& out, const LsaHandle& h) {
  const std::size_t start = out.size();
  write_lsa_header_fields(out, 0, h.origin, h.seq,
                          static_cast<std::uint16_t>(kLsaHeaderSize));
  patch_lsa_checksum(out, start);
}

void put_router_lsa(Bytes& out, const Lsa& lsa) {
  const std::size_t start = out.size();
  const std::uint16_t length = static_cast<std::uint16_t>(
      kLsaHeaderSize + 4 + 12 * lsa.links.size());
  write_lsa_header_fields(out, lsa.age_sec, lsa.origin, lsa.seq, length);
  put_u8(out, 0);  // flags
  put_u8(out, 0);
  put_u16_be(out, static_cast<std::uint16_t>(lsa.links.size()));
  for (const Adjacency& a : lsa.links) {
    put_u32_be(out, a.peer);  // Link ID = neighbour Router ID
    put_u32_be(out, 0);       // Link Data: unnumbered
    put_u8(out, kOspfLinkTypeP2P);
    put_u8(out, 0);  // # TOS
    const std::uint16_t metric =
        static_cast<std::uint16_t>(std::min<Cost>(a.cost, 0xFFFFu));
    put_u16_be(out, metric);
  }
  patch_lsa_checksum(out, start);
}

bool read_lsa_handle(Reader& r, LsaHandle& out) {
  if (r.remaining() < kLsaHeaderSize) return false;
  const ByteView raw = r.take(kLsaHeaderSize);
  if (r.bad()) return false;
  Reader hr(raw);
  (void)hr.u16();  // age
  (void)hr.u8();   // options
  const std::uint8_t type = hr.u8();
  const NodeId lsid = hr.u32();
  const NodeId adv = hr.u32();
  const std::uint32_t seq = hr.u32();
  const std::uint16_t csum = hr.u16();
  const std::uint16_t length = hr.u16();
  if (hr.bad()) return false;
  if (type != kOspfLsTypeRouter) return false;
  if (lsid != adv) return false;
  if (length < kLsaHeaderSize) return false;
  if (ospf_lsa_checksum(raw) != csum) return false;
  out.origin = adv;
  out.seq = seq;
  return true;
}

// Assemble an OSPFv2 header with a placeholder checksum, append the body,
// then patch length and checksum.
Bytes finish_packet(PacketType type, NodeId sender, const Bytes& body) {
  Bytes out;
  out.reserve(kHeaderSize + body.size());
  put_u8(out, kProtocolVersion);
  put_u8(out, static_cast<std::uint8_t>(type));
  put_u16_be(out, 0);  // length, patched below
  put_u32_be(out, sender);
  put_u32_be(out, kOspfAreaId);
  put_u16_be(out, 0);  // checksum, patched below
  put_u16_be(out, kOspfAuTypeNull);
  for (int i = 0; i < 8; ++i) put_u8(out, 0);  // authentication
  out.insert(out.end(), body.begin(), body.end());

  const std::uint16_t len = static_cast<std::uint16_t>(out.size());
  out[2] = static_cast<std::uint8_t>((len >> 8) & 0xFF);
  out[3] = static_cast<std::uint8_t>(len & 0xFF);

  const std::uint16_t csum = ospf_packet_checksum(out);
  out[12] = static_cast<std::uint8_t>((csum >> 8) & 0xFF);
  out[13] = static_cast<std::uint8_t>(csum & 0xFF);
  return out;
}

std::optional<Reader> open_body(ByteView data, PacketType expected) {
  const std::optional<Header> h = decode_header(data);
  if (!h || h->type != expected) return std::nullopt;
  Reader r(data);
  // Skip the 24-byte header.
  for (std::size_t i = 0; i < kHeaderSize; ++i) r.u8();
  if (r.bad()) return std::nullopt;
  return r;
}

std::uint16_t seconds_from_ms(std::uint32_t ms) {
  return static_cast<std::uint16_t>(std::min<std::uint32_t>(ms / 1000u, 0xFFFFu));
}

}  // namespace

std::size_t lsa_wire_size(const Lsa& lsa) {
  return kLsaHeaderSize + 4 + 12 * lsa.links.size();
}

Bytes encode_hello(NodeId sender, const Hello& h) {
  Bytes body;
  put_u32_be(body, 0);  // Network Mask (point-to-point / unnumbered)
  put_u16_be(body, seconds_from_ms(h.hello_interval_ms));
  put_u8(body, kOspfOptionE);
  put_u8(body, 0);  // Router Priority (0: never elected DR)
  put_u32_be(body, h.dead_interval_ms / 1000u);
  put_u32_be(body, 0);  // Designated Router
  put_u32_be(body, 0);  // Backup Designated Router
  for (NodeId n : h.seen) put_u32_be(body, n);
  return finish_packet(PacketType::Hello, sender, body);
}

Bytes encode_db_description(NodeId sender, const DbDescription& d) {
  Bytes body;
  put_u16_be(body, 1500);  // Interface MTU
  put_u8(body, kOspfOptionE);
  // Unsolicited summary: Init + Master. Reply: Slave, not Init.
  const std::uint8_t flags =
      d.reply ? std::uint8_t{0}
              : static_cast<std::uint8_t>(kOspfDdFlagI | kOspfDdFlagMs);
  put_u8(body, flags);
  put_u32_be(body, 1);  // DD sequence number
  for (const LsaHandle& s : d.summary) put_lsa_header(body, s);
  return finish_packet(PacketType::DbDescription, sender, body);
}

Bytes encode_ls_request(NodeId sender, const LsRequest& req) {
  Bytes body;
  for (NodeId origin : req.origins) {
    put_u32_be(body, kOspfLsTypeRouter);
    put_u32_be(body, origin);  // Link State ID
    put_u32_be(body, origin);  // Advertising Router
  }
  return finish_packet(PacketType::LsRequest, sender, body);
}

Bytes encode_ls_update(NodeId sender, const LsUpdate& u) {
  Bytes body;
  put_u32_be(body, static_cast<std::uint32_t>(u.lsas.size()));
  for (const Lsa& l : u.lsas) put_router_lsa(body, l);
  return finish_packet(PacketType::LsUpdate, sender, body);
}

Bytes encode_ls_ack(NodeId sender, const LsAck& a) {
  Bytes body;
  for (const LsaHandle& s : a.acked) put_lsa_header(body, s);
  return finish_packet(PacketType::LsAck, sender, body);
}

std::optional<Header> decode_header(ByteView data) {
  if (data.size() < kHeaderSize) return std::nullopt;
  Reader r(data);
  Header h;
  h.version = r.u8();
  const std::uint8_t type = r.u8();
  h.length = r.u16();
  h.sender = r.u32();
  h.area = r.u32();
  h.checksum = r.u16();
  h.au_type = r.u16();
  // Skip 8-byte authentication.
  for (int i = 0; i < 8; ++i) r.u8();
  if (r.bad()) return std::nullopt;
  if (h.version != kProtocolVersion) return std::nullopt;
  if (type < 1 || type > 5) return std::nullopt;
  h.type = static_cast<PacketType>(type);
  if (h.length != data.size()) return std::nullopt;
  if (h.au_type != kOspfAuTypeNull) return std::nullopt;
  if (ospf_packet_checksum(data) != h.checksum) return std::nullopt;
  return h;
}

std::optional<Hello> decode_hello(ByteView data) {
  auto reader = open_body(data, PacketType::Hello);
  if (!reader) return std::nullopt;
  Reader& r = *reader;
  Hello h;
  (void)r.u32();  // Network Mask
  const std::uint16_t hello_s = r.u16();
  (void)r.u8();  // Options
  (void)r.u8();  // Priority
  const std::uint32_t dead_s = r.u32();
  (void)r.u32();  // DR
  (void)r.u32();  // BDR
  if (r.bad()) return std::nullopt;
  h.hello_interval_ms = static_cast<std::uint32_t>(hello_s) * 1000u;
  h.dead_interval_ms = dead_s * 1000u;
  if (r.remaining() % 4 != 0) return std::nullopt;
  while (r.remaining() > 0) h.seen.push_back(r.u32());
  if (r.bad()) return std::nullopt;
  return h;
}

std::optional<DbDescription> decode_db_description(ByteView data) {
  auto reader = open_body(data, PacketType::DbDescription);
  if (!reader) return std::nullopt;
  Reader& r = *reader;
  DbDescription d;
  (void)r.u16();  // MTU
  (void)r.u8();   // Options
  const std::uint8_t flags = r.u8();
  (void)r.u32();  // DD sequence
  if (r.bad()) return std::nullopt;
  d.reply = (flags & kOspfDdFlagMs) == 0;
  if (r.remaining() % kLsaHeaderSize != 0) return std::nullopt;
  while (r.remaining() > 0) {
    LsaHandle s;
    if (!read_lsa_handle(r, s)) return std::nullopt;
    d.summary.push_back(s);
  }
  return d;
}

std::optional<LsRequest> decode_ls_request(ByteView data) {
  auto reader = open_body(data, PacketType::LsRequest);
  if (!reader) return std::nullopt;
  Reader& r = *reader;
  LsRequest req;
  if (r.remaining() % 12 != 0) return std::nullopt;
  while (r.remaining() > 0) {
    const std::uint32_t type = r.u32();
    const NodeId lsid = r.u32();
    const NodeId adv = r.u32();
    if (r.bad()) return std::nullopt;
    if (type != kOspfLsTypeRouter) return std::nullopt;
    if (lsid != adv) return std::nullopt;
    req.origins.push_back(adv);
  }
  return req;
}

std::optional<LsUpdate> decode_ls_update(ByteView data) {
  const auto header = decode_header(data);
  if (!header || header->type != PacketType::LsUpdate) return std::nullopt;
  if (data.size() < kHeaderSize + 4) return std::nullopt;

  const std::uint32_t count = (static_cast<std::uint32_t>(data[kHeaderSize]) << 24) |
                              (static_cast<std::uint32_t>(data[kHeaderSize + 1]) << 16) |
                              (static_cast<std::uint32_t>(data[kHeaderSize + 2]) << 8) |
                              static_cast<std::uint32_t>(data[kHeaderSize + 3]);
  // Each LSA is at least a header plus the flags/#links word.
  if (static_cast<std::uint64_t>(count) * (kLsaHeaderSize + 4) >
      data.size() - (kHeaderSize + 4)) {
    return std::nullopt;
  }

  LsUpdate u;
  u.lsas.reserve(count);
  std::size_t off = kHeaderSize + 4;
  for (std::uint32_t i = 0; i < count; ++i) {
    if (off + kLsaHeaderSize > data.size()) return std::nullopt;
    const std::uint16_t length = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[off + 18]) << 8) | data[off + 19]);
    if (length < kLsaHeaderSize + 4) return std::nullopt;
    if (off + length > data.size()) return std::nullopt;
    const ByteView lsa_bytes = data.subspan(off, length);
    const std::uint16_t declared_csum = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(lsa_bytes[16]) << 8) | lsa_bytes[17]);
    if (ospf_lsa_checksum(lsa_bytes) != declared_csum) return std::nullopt;

    Reader lr(lsa_bytes);
    Lsa lsa;
    lsa.age_sec = lr.u16();
    (void)lr.u8();  // options
    const std::uint8_t type = lr.u8();
    const NodeId lsid = lr.u32();
    const NodeId adv = lr.u32();
    lsa.seq = lr.u32();
    (void)lr.u16();  // checksum already verified
    (void)lr.u16();  // length
    if (lr.bad() || type != kOspfLsTypeRouter || lsid != adv) return std::nullopt;
    lsa.origin = adv;

    (void)lr.u8();  // flags
    (void)lr.u8();
    const std::uint16_t nlinks = lr.u16();
    if (lr.bad()) return std::nullopt;
    if (static_cast<std::uint64_t>(nlinks) * 12 > lr.remaining()) return std::nullopt;
    lsa.links.reserve(nlinks);
    for (std::uint16_t j = 0; j < nlinks; ++j) {
      const NodeId peer = lr.u32();
      (void)lr.u32();  // Link Data
      const std::uint8_t link_type = lr.u8();
      const std::uint8_t tos_count = lr.u8();
      const std::uint16_t metric = lr.u16();
      if (lr.bad()) return std::nullopt;
      for (std::uint8_t t = 0; t < tos_count; ++t) {
        (void)lr.u8();
        (void)lr.u8();
        (void)lr.u16();
      }
      if (lr.bad()) return std::nullopt;
      if (link_type == kOspfLinkTypeP2P) {
        lsa.links.push_back(Adjacency{peer, metric});
      }
    }
    if (lr.bad() || lr.remaining() != 0) return std::nullopt;
    u.lsas.push_back(std::move(lsa));
    off += length;
  }
  if (off != data.size()) return std::nullopt;
  return u;
}

std::optional<LsAck> decode_ls_ack(ByteView data) {
  auto reader = open_body(data, PacketType::LsAck);
  if (!reader) return std::nullopt;
  Reader& r = *reader;
  LsAck a;
  if (r.remaining() % kLsaHeaderSize != 0) return std::nullopt;
  while (r.remaining() > 0) {
    LsaHandle s;
    if (!read_lsa_handle(r, s)) return std::nullopt;
    a.acked.push_back(s);
  }
  return a;
}

}  // namespace waypoint
