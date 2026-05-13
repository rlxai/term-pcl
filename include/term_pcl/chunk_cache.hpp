#ifndef TERM_PCL_CHUNK_CACHE_HPP_
#define TERM_PCL_CHUNK_CACHE_HPP_

#include "term_pcl/types.hpp"

#include <cstddef>
#include <list>
#include <optional>
#include <unordered_map>
#include <vector>

namespace term_pcl {

class ChunkCache {
public:
    explicit ChunkCache(std::size_t max_points);

    void put(std::size_t chunk_id, std::vector<term_pcl::Point> points);
    std::optional<std::vector<term_pcl::Point>> get(std::size_t chunk_id);
    std::size_t cached_chunks() const;
    std::size_t cached_points() const;

private:
    struct Entry {
        std::vector<term_pcl::Point> points;
        std::list<std::size_t>::iterator lru_it;
    };

    void touch(std::size_t chunk_id, Entry& entry);
    void evict_to_budget();

    std::size_t max_points_ = 0;
    std::size_t cached_points_ = 0;
    std::list<std::size_t> lru_;
    std::unordered_map<std::size_t, Entry> entries_;
};

}  // namespace term_pcl

#endif  // TERM_PCL_CHUNK_CACHE_HPP_
