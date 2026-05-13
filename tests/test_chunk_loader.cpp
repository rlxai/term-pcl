#include "term_pcl/chunk_loader.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

using term_pcl::ChunkLoader;
using term_pcl::Point;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void write_chunk(const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::binary);
    Point p1{1.0f, 2.0f, 3.0f};
    Point p2{4.0f, 5.0f, 6.0f};
    out.write(reinterpret_cast<const char*>(&p1), sizeof(Point));
    out.write(reinterpret_cast<const char*>(&p2), sizeof(Point));
}

void test_loader_reads_chunk_on_worker_thread() {
    const auto dir = std::filesystem::temp_directory_path() / "term_pcl_chunk_loader_test";
    std::filesystem::create_directories(dir);
    const auto chunk_path = dir / "chunk.bin";
    write_chunk(chunk_path);

    ChunkLoader loader(1);
    require(loader.request(3, chunk_path.string()), "First request should be queued");

    bool loaded = false;
    for (int i = 0; i < 50; ++i) {
        auto result = loader.take_completed();
        if (result.has_value()) {
            require(result->chunk_id == 3, "Loaded chunk id should match request");
            require(result->points.size() == 2, "Loaded chunk should contain binary points");
            loaded = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    require(loaded, "Chunk should complete asynchronously");
    std::filesystem::remove_all(dir);
}

void test_loader_deduplicates_in_flight_requests() {
    const auto dir = std::filesystem::temp_directory_path() / "term_pcl_chunk_loader_dedupe_test";
    std::filesystem::create_directories(dir);
    const auto chunk_path = dir / "chunk.bin";
    write_chunk(chunk_path);

    ChunkLoader loader(1);
    require(loader.request(4, chunk_path.string()), "First request should be queued");
    require(!loader.request(4, chunk_path.string()), "Duplicate in-flight request should not be queued");
    std::filesystem::remove_all(dir);
}

void test_loader_reports_missing_chunk() {
    ChunkLoader loader(1);
    require(loader.request(5, "/tmp/term-pcl-missing-chunk.bin"), "Missing chunk request should be queued");

    bool loaded = false;
    for (int i = 0; i < 50; ++i) {
        auto result = loader.take_completed();
        if (result.has_value()) {
            require(result->chunk_id == 5, "Missing chunk id should match request");
            require(!result->ok, "Missing chunk should report failure");
            require(result->error.find("Failed to read chunk") != std::string::npos,
                    "Missing chunk error should explain read failure");
            loaded = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    require(loaded, "Missing chunk should complete asynchronously");
}

void test_loader_reports_truncated_chunk() {
    const auto dir = std::filesystem::temp_directory_path() / "term_pcl_chunk_loader_truncated_test";
    std::filesystem::create_directories(dir);
    const auto chunk_path = dir / "chunk.bin";
    std::ofstream(chunk_path, std::ios::binary) << "x";

    ChunkLoader loader(1);
    require(loader.request(6, chunk_path.string()), "Truncated chunk request should be queued");

    bool loaded = false;
    for (int i = 0; i < 50; ++i) {
        auto result = loader.take_completed();
        if (result.has_value()) {
            require(result->chunk_id == 6, "Truncated chunk id should match request");
            require(!result->ok, "Truncated chunk should report failure");
            require(result->error.find("Malformed chunk") != std::string::npos,
                    "Truncated chunk error should explain malformed payload");
            loaded = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    require(loaded, "Truncated chunk should complete asynchronously");
    std::filesystem::remove_all(dir);
}

int main() {
    test_loader_reads_chunk_on_worker_thread();
    test_loader_deduplicates_in_flight_requests();
    test_loader_reports_missing_chunk();
    test_loader_reports_truncated_chunk();
    return 0;
}
