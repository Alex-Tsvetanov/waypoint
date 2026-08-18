#include "waypoint/lsdb.hpp"

#include <algorithm>

namespace waypoint {
namespace {

std::uint16_t aged(std::uint16_t at_install, Micros installed_at, Micros now) {
  const Micros elapsed = now > installed_at ? now - installed_at : 0;
  const Micros total = static_cast<Micros>(at_install) + elapsed / kSec;
  return static_cast<std::uint16_t>(std::min<Micros>(total, kMaxAgeSec));
}

}  // namespace

Lsdb::Result Lsdb::install(const Lsa& lsa, Micros now) {
  const auto it = entries_.find(lsa.origin);
  if (it == entries_.end()) {
    // Nothing to withdraw. Returning Ignored rather than Expired is what stops
    // a maximum age advertisement from being reflooded around a cycle for ever:
    // the second node to see it holds nothing either, so it must not pass it on.
    if (lsa.age_sec >= kMaxAgeSec) return Result::Ignored;
    entries_.emplace(lsa.origin, Entry{lsa, now});
    return Result::Installed;
  }

  if (seq_newer(lsa.seq, it->second.lsa.seq)) {
    if (lsa.age_sec >= kMaxAgeSec) {
      // A maximum age advertisement is how an origin withdraws itself. It is
      // newer than what we hold, so it wins, and what it says is "forget me".
      entries_.erase(it);
      return Result::Expired;
    }
    it->second = Entry{lsa, now};
    return Result::Installed;
  }
  if (lsa.seq == it->second.lsa.seq) return Result::Duplicate;
  return Result::Older;
}

const Lsa* Lsdb::find(NodeId origin) const {
  const auto it = entries_.find(origin);
  return it == entries_.end() ? nullptr : &it->second.lsa;
}

std::uint16_t Lsdb::age_of(NodeId origin, Micros now) const {
  const auto it = entries_.find(origin);
  if (it == entries_.end()) return kMaxAgeSec;
  return aged(it->second.lsa.age_sec, it->second.installed_at, now);
}

std::vector<LsaHandle> Lsdb::summary() const {
  std::vector<LsaHandle> out;
  out.reserve(entries_.size());
  for (const auto& [origin, entry] : entries_) {
    out.push_back(LsaHandle{origin, entry.lsa.seq});
  }
  return out;
}

std::optional<Lsa> Lsdb::get(NodeId origin, Micros now) const {
  const auto it = entries_.find(origin);
  if (it == entries_.end()) return std::nullopt;
  Lsa copy = it->second.lsa;
  copy.age_sec = aged(copy.age_sec, it->second.installed_at, now);
  return copy;
}

std::vector<Lsa> Lsdb::snapshot(Micros now) const {
  std::vector<Lsa> out;
  out.reserve(entries_.size());
  for (const auto& [origin, entry] : entries_) {
    Lsa copy = entry.lsa;
    copy.age_sec = aged(copy.age_sec, entry.installed_at, now);
    out.push_back(std::move(copy));
  }
  return out;
}

std::size_t Lsdb::purge_expired(Micros now) {
  std::size_t removed = 0;
  for (auto it = entries_.begin(); it != entries_.end();) {
    if (aged(it->second.lsa.age_sec, it->second.installed_at, now) >= kMaxAgeSec) {
      it = entries_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }
  return removed;
}

Graph Lsdb::graph() const {
  Graph g;
  for (const auto& [origin, entry] : entries_) {
    g.adj.try_emplace(origin);
    for (const Adjacency& a : entry.lsa.links) g.add_edge(origin, a.peer, a.cost);
  }
  return g;
}

}  // namespace waypoint
