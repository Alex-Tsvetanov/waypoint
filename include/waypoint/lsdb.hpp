// The link state database: one advertisement per originating router, kept
// current by sequence number and expired by age.
#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <vector>

#include "waypoint/packet.hpp"
#include "waypoint/spf.hpp"
#include "waypoint/types.hpp"

namespace waypoint {

class Lsdb {
 public:
  enum class Result {
    Installed,  // strictly newer than what was held, stored, must be reflooded
    Duplicate,  // same sequence number, acknowledge but do not reflood
    Older,      // stale, the receiver holds something newer
    Expired,    // at maximum age and newer than what we held: a withdrawal
    Ignored,    // at maximum age for an origin we do not hold, nothing to do
  };

  Result install(const Lsa& lsa, Micros now);

  const Lsa* find(NodeId origin) const;
  std::size_t size() const { return entries_.size(); }
  bool empty() const { return entries_.empty(); }

  // Age carried on the wire is the age at origination plus the time the entry
  // has been held here. Ageing is derived from the install timestamp rather
  // than driven by a periodic timer, so it costs nothing between events and
  // stays exact under a virtual clock.
  std::uint16_t age_of(NodeId origin, Micros now) const;

  std::vector<LsaHandle> summary() const;
  std::vector<Lsa> snapshot(Micros now) const;
  std::optional<Lsa> get(NodeId origin, Micros now) const;

  // Drops every entry that has reached the maximum age. Returns how many went.
  std::size_t purge_expired(Micros now);

  // The directed graph advertised by the database. Call `bidirectional()` on
  // the result before running the shortest path computation.
  Graph graph() const;

 private:
  struct Entry {
    Lsa lsa;
    Micros installed_at = 0;
  };
  std::map<NodeId, Entry> entries_;
};

}  // namespace waypoint
