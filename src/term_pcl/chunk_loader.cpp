#include "term_pcl/chunk_loader.hpp"

#include <fstream>
#include <stdexcept>
#include <utility>

namespace term_pcl {

ChunkLoader::ChunkLoader(std::size_t worker_count) {
    const std::size_t count = worker_count == 0 ? 1 : worker_count;
    for (std::size_t i = 0; i < count; ++i) {
        workers_.emplace_back([this]() { worker_loop(); });
    }
}

ChunkLoader::~ChunkLoader() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto& worker : workers_) {
        if (worker.joinable()) worker.join();
    }
}

bool ChunkLoader::request(std::size_t chunk_id, std::string path) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (in_flight_.count(chunk_id) > 0) return false;
        in_flight_.insert(chunk_id);
        pending_.push(Request{chunk_id, std::move(path)});
    }
    cv_.notify_one();
    return true;
}

std::optional<LoadedChunk> ChunkLoader::take_completed() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (completed_.empty()) return std::nullopt;
    auto result = std::move(completed_.front());
    completed_.pop();
    return result;
}

std::size_t ChunkLoader::queued_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_.size() + in_flight_.size();
}

void ChunkLoader::worker_loop() {
    while (true) {
        Request request;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() { return stop_ || !pending_.empty(); });
            if (stop_ && pending_.empty()) return;
            request = std::move(pending_.front());
            pending_.pop();
        }

        LoadedChunk loaded;
        loaded.chunk_id = request.chunk_id;
        try {
            loaded.points = read_chunk_file(request.path);
        } catch (const std::exception& error) {
            loaded.ok = false;
            loaded.error = error.what();
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            completed_.push(std::move(loaded));
            in_flight_.erase(request.chunk_id);
        }
    }
}

std::vector<term_pcl::Point> ChunkLoader::read_chunk_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) {
        throw std::runtime_error("Failed to read chunk: " + path);
    }
    const auto size = in.tellg();
    if (size < 0) {
        throw std::runtime_error("Malformed chunk: " + path);
    }
    const auto byte_count = static_cast<std::size_t>(size);
    if (byte_count % sizeof(term_pcl::Point) != 0) {
        throw std::runtime_error("Malformed chunk: " + path);
    }
    in.seekg(0);

    std::vector<term_pcl::Point> points(byte_count / sizeof(term_pcl::Point));
    in.read(reinterpret_cast<char*>(points.data()), static_cast<std::streamsize>(byte_count));
    if (!in) {
        throw std::runtime_error("Malformed chunk: " + path);
    }
    return points;
}

}  // namespace term_pcl
