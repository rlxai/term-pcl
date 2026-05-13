#include "term_pcl/chunk_cache.hpp"

#include <stdexcept>
#include <string>
#include <vector>

using term_pcl::ChunkCache;
using term_pcl::Point;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<Point> points(std::size_t count) {
    std::vector<Point> result;
    for (std::size_t i = 0; i < count; ++i) {
        result.push_back(Point{static_cast<float>(i), 0.0f, 0.0f});
    }
    return result;
}

void test_cache_returns_inserted_chunk() {
    ChunkCache cache(10);
    cache.put(7, points(3));

    const auto chunk = cache.get(7);
    require(chunk.has_value(), "Inserted chunk should be returned");
    require(chunk->size() == 3, "Returned chunk should preserve point count");
    require(cache.cached_chunks() == 1, "Cache should report one cached chunk");
}

void test_cache_evicts_least_recently_used_chunk() {
    ChunkCache cache(5);
    cache.put(1, points(2));
    cache.put(2, points(2));
    require(cache.get(1).has_value(), "Accessing chunk 1 should make it recently used");
    cache.put(3, points(2));

    require(cache.get(1).has_value(), "Recently used chunk should remain cached");
    require(!cache.get(2).has_value(), "Least recently used chunk should be evicted");
    require(cache.get(3).has_value(), "Newest chunk should be cached");
}

int main() {
    test_cache_returns_inserted_chunk();
    test_cache_evicts_least_recently_used_chunk();
    return 0;
}
