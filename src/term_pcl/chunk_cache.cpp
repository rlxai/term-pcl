#include "term_pcl/chunk_cache.hpp"

#include <utility>

namespace term_pcl {

ChunkCache::ChunkCache(std::size_t max_points) : max_points_(max_points) {}

void ChunkCache::touch(std::size_t chunk_id, Entry& entry) {
    lru_.erase(entry.lru_it);
    lru_.push_front(chunk_id);
    entry.lru_it = lru_.begin();
}

void ChunkCache::put(std::size_t chunk_id, std::vector<term_pcl::Point> points) {
    const auto existing = entries_.find(chunk_id);
    if (existing != entries_.end()) {
        cached_points_ -= existing->second.points.size();
        existing->second.points = std::move(points);
        cached_points_ += existing->second.points.size();
        touch(chunk_id, existing->second);
        evict_to_budget();
        return;
    }

    lru_.push_front(chunk_id);
    cached_points_ += points.size();
    entries_.emplace(chunk_id, Entry{std::move(points), lru_.begin()});
    evict_to_budget();
}

std::optional<std::vector<term_pcl::Point>> ChunkCache::get(std::size_t chunk_id) {
    auto found = entries_.find(chunk_id);
    if (found == entries_.end()) return std::nullopt;
    touch(chunk_id, found->second);
    return found->second.points;
}

std::size_t ChunkCache::cached_chunks() const {
    return entries_.size();
}

std::size_t ChunkCache::cached_points() const {
    return cached_points_;
}

void ChunkCache::evict_to_budget() {
    while (cached_points_ > max_points_ && !lru_.empty()) {
        const auto victim = lru_.back();
        lru_.pop_back();
        auto found = entries_.find(victim);
        if (found != entries_.end()) {
            cached_points_ -= found->second.points.size();
            entries_.erase(found);
        }
    }
}

}  // namespace term_pcl
