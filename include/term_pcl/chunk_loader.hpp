#ifndef TERM_PCL_CHUNK_LOADER_HPP_
#define TERM_PCL_CHUNK_LOADER_HPP_

#include "term_pcl/types.hpp"

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace term_pcl {

struct LoadedChunk {
    std::size_t chunk_id = 0;
    std::vector<term_pcl::Point> points;
    bool ok = true;
    std::string error;
};

class ChunkLoader {
public:
    explicit ChunkLoader(std::size_t worker_count);
    ~ChunkLoader();

    ChunkLoader(const ChunkLoader&) = delete;
    ChunkLoader& operator=(const ChunkLoader&) = delete;

    bool request(std::size_t chunk_id, std::string path);
    std::optional<LoadedChunk> take_completed();
    std::size_t queued_count() const;

private:
    struct Request {
        std::size_t chunk_id = 0;
        std::string path;
    };

    void worker_loop();
    static std::vector<term_pcl::Point> read_chunk_file(const std::string& path);

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_ = false;
    std::queue<Request> pending_;
    std::queue<LoadedChunk> completed_;
    std::unordered_set<std::size_t> in_flight_;
    std::vector<std::thread> workers_;
};

}  // namespace term_pcl

#endif  // TERM_PCL_CHUNK_LOADER_HPP_
