#include "term_pcl/cloud_optimizer.hpp"
#include "term_pcl/point_cloud_loader.hpp"
#include "term_pcl/termcloud_index.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using term_pcl::CloudData;
using term_pcl::Point;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_close(float actual, float expected, const std::string& message) {
    require(std::fabs(actual - expected) < 0.0001f, message);
}

void require_load_error_contains(const std::string& expected, const std::function<void()>& operation) {
    try {
        operation();
    } catch (const term_pcl::LoadError& error) {
        require(std::string(error.what()).find(expected) != std::string::npos,
                "LoadError should contain: " + expected);
        return;
    }
    throw std::runtime_error("expected LoadError containing: " + expected);
}

void test_termcloud_v1_reader_compatibility() {
    const std::string output = "build/test-termcloud/v1.termcloud";
    std::filesystem::remove_all(output);
    std::filesystem::create_directories(output);

    CloudData cloud;
    cloud.frame_id = "legacy.xyz";
    cloud.points = {Point{1.0f, 2.0f, 3.0f}, Point{4.0f, 5.0f, 6.0f}};
    term_pcl::recomputeStats(cloud);

    {
        std::ofstream metadata(output + "/metadata.txt");
        metadata << "termcloud 1\n"
                 << cloud.frame_id << "\n"
                 << cloud.points.size() << "\n"
                 << cloud.min_x << ' ' << cloud.max_x << "\n"
                 << cloud.min_y << ' ' << cloud.max_y << "\n"
                 << cloud.min_z << ' ' << cloud.max_z << "\n"
                 << cloud.cx << ' ' << cloud.cy << ' ' << cloud.cz << "\n";
    }
    {
        std::ofstream points(output + "/points.bin", std::ios::binary);
        points.write(reinterpret_cast<const char*>(cloud.points.data()),
                     static_cast<std::streamsize>(cloud.points.size() * sizeof(Point)));
    }

    auto loaded = term_pcl::readTermcloudIndex(output);
    require(loaded->frame_id == "legacy.xyz", "termcloud should preserve v1 frame id");
    require(loaded->points.size() == 2, "termcloud should preserve v1 point count");
    require_close(loaded->points[1].x, 4.0f, "termcloud should preserve v1 point data");
}

void test_legacy_v1_xyz_only_points_bin_loads_uncolored_points() {
    const std::string output = "build/test-termcloud/v1_xyz_only.termcloud";
    std::filesystem::remove_all(output);
    std::filesystem::create_directories(output);

    const std::vector<float> xyz_records = {
        1.0f, 2.0f, 3.0f,
        4.0f, 5.0f, 6.0f,
    };
    {
        std::ofstream metadata(output + "/metadata.txt");
        metadata << "termcloud 1\n"
                 << "legacy_xyz_only.xyz\n"
                 << "2\n"
                 << "1 4\n"
                 << "2 5\n"
                 << "3 6\n"
                 << "2.5 3.5 4.5\n";
    }
    {
        std::ofstream points(output + "/points.bin", std::ios::binary);
        points.write(reinterpret_cast<const char*>(xyz_records.data()),
                     static_cast<std::streamsize>(xyz_records.size() * sizeof(float)));
    }

    auto loaded = term_pcl::readTermcloudIndex(output);
    require(loaded->points.size() == 2, "legacy v1 XYZ-only payload should load two points");
    require_close(loaded->points[0].x, 1.0f, "legacy v1 XYZ-only payload should preserve first x");
    require_close(loaded->points[1].z, 6.0f, "legacy v1 XYZ-only payload should preserve second z");
    require(!loaded->points[0].has_color, "legacy v1 XYZ-only point should be uncolored");
    require(!loaded->points[1].has_color, "legacy v1 XYZ-only point should be uncolored");
}

CloudData make_cloud(std::size_t count) {
    CloudData cloud;
    cloud.frame_id = "generated.xyz";
    for (std::size_t i = 0; i < count; ++i) {
        cloud.points.push_back(Point{static_cast<float>(i), static_cast<float>(i % 7), 0.0f});
    }
    term_pcl::recomputeStats(cloud);
    return cloud;
}

void test_termcloud_v2_writes_chunk_directory() {
    const std::string output = "build/test-termcloud/chunked.termcloud";
    std::filesystem::remove_all(output);

    auto cloud = make_cloud(9000);
    cloud.frame_id = "chunked.xyz";

    term_pcl::writeTermcloudIndex(cloud, output);

    require(std::filesystem::exists(output + "/metadata.txt"), "termcloud should write metadata");
    require(std::filesystem::exists(output + "/chunks/000000.bin"), "termcloud should write first chunk");
    require(std::filesystem::exists(output + "/chunks/000001.bin"), "termcloud should write second chunk");

    auto loaded = term_pcl::readTermcloudIndex(output);
    require(loaded->frame_id == "chunked.xyz", "termcloud should preserve v2 frame id");
    require(loaded->points.size() == 9000, "termcloud should preserve v2 point count");
}

void test_legacy_v2_xyz_only_chunks_load_uncolored_points() {
    const std::string output = "build/test-termcloud/v2_xyz_only.termcloud";
    std::filesystem::remove_all(output);
    std::filesystem::create_directories(output + "/chunks");

    const std::vector<float> chunk0 = {
        -1.0f, 0.0f, 1.0f,
    };
    const std::vector<float> chunk1 = {
        2.0f, 3.0f, 4.0f,
    };
    {
        std::ofstream metadata(output + "/metadata.txt");
        metadata << "termcloud 2\n"
                 << "legacy_v2_xyz_only.xyz\n"
                 << "2\n"
                 << "-1 2\n"
                 << "0 3\n"
                 << "1 4\n"
                 << "0.5 1.5 2.5\n";
    }
    {
        std::ofstream points(output + "/chunks/000000.bin", std::ios::binary);
        points.write(reinterpret_cast<const char*>(chunk0.data()),
                     static_cast<std::streamsize>(chunk0.size() * sizeof(float)));
    }
    {
        std::ofstream points(output + "/chunks/000001.bin", std::ios::binary);
        points.write(reinterpret_cast<const char*>(chunk1.data()),
                     static_cast<std::streamsize>(chunk1.size() * sizeof(float)));
    }

    auto loaded = term_pcl::readTermcloudIndex(output);
    require(loaded->points.size() == 2, "legacy v2 XYZ-only chunks should load two points");
    require_close(loaded->points[0].x, -1.0f, "legacy v2 XYZ-only chunks should preserve first x");
    require_close(loaded->points[1].z, 4.0f, "legacy v2 XYZ-only chunks should preserve second z");
    require(!loaded->points[0].has_color, "legacy v2 XYZ-only chunk point should be uncolored");
    require(!loaded->points[1].has_color, "legacy v2 XYZ-only chunk point should be uncolored");
}

void test_v3_writer_creates_hierarchy_metadata_and_node_chunks() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_v3_writer_test.termcloud";
    std::filesystem::remove_all(dir);

    auto cloud = make_cloud(10);
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), 3, 3);

    require(std::filesystem::exists(dir / "metadata.txt"), "v3 metadata should exist");
    require(std::filesystem::exists(dir / "nodes.txt"), "v3 node metadata should exist");
    require(std::filesystem::exists(dir / "chunks"), "v3 chunks directory should exist");

    const auto metadata = term_pcl::readTermcloudHierarchyMetadata(dir.string());
    require(metadata.index.point_count == 10, "v3 metadata should preserve point count");
    require(metadata.index.nodes.size() > 1, "v3 metadata should include hierarchy nodes");
    require(!metadata.base_path.empty(), "v3 metadata should remember base path for lazy chunk loading");

    std::filesystem::remove_all(dir);
}

void test_termcloud_v3_reader_accepts_metadata_without_has_rgb() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_v3_legacy_metadata_test.termcloud";
    std::filesystem::remove_all(dir);

    auto cloud = make_cloud(1);
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), 3, 3);

    {
        std::ofstream metadata(dir / "metadata.txt");
        metadata << "termcloud 3\n"
                 << cloud.frame_id << "\n"
                 << cloud.points.size() << "\n"
                 << cloud.min_x << ' ' << cloud.max_x << "\n"
                 << cloud.min_y << ' ' << cloud.max_y << "\n"
                 << cloud.min_z << ' ' << cloud.max_z << "\n"
                 << cloud.cx << ' ' << cloud.cy << ' ' << cloud.cz << "\n"
                 << "1\n";
    }

    const auto metadata = term_pcl::readTermcloudHierarchyMetadata(dir.string());
    require(metadata.index.point_count == 1, "legacy v3 metadata should preserve point count without has_rgb");
    require(metadata.index.nodes.size() == 1, "legacy v3 metadata should load nodes without has_rgb");

    std::filesystem::remove_all(dir);
}

void test_termcloud_v3_preserves_rgb_points() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_v3_rgb_test.termcloud";
    std::filesystem::remove_all(dir);

    CloudData cloud;
    cloud.frame_id = "colored.xyzrgb";
    Point point{1.0f, 2.0f, 3.0f};
    point.has_color = true;
    point.r = 37;
    point.g = 216;
    point.b = 101;
    cloud.points = {point};
    term_pcl::recomputeStats(cloud);

    term_pcl::writeTermcloudV3Index(cloud, dir.string(), 3, 3);

    std::ifstream metadata_file(dir / "metadata.txt");
    std::vector<std::string> metadata_lines;
    for (std::string line; std::getline(metadata_file, line);) {
        metadata_lines.push_back(line);
    }
    require(metadata_lines.size() >= 9, "v3 RGB metadata should include has_rgb marker");
    require(metadata_lines[8] == "has_rgb 1", "v3 RGB metadata should mark RGB payloads");

    const auto metadata = term_pcl::readTermcloudHierarchyMetadata(dir.string());
    require(metadata.index.point_count == 1, "v3 RGB metadata should preserve point count");
    require(metadata.index.nodes.size() == 1, "v3 RGB metadata should include one node");

    const auto chunk_path = dir / metadata.index.nodes.front().chunk_file;
    std::ifstream chunk(chunk_path, std::ios::binary);
    require(static_cast<bool>(chunk), "v3 RGB chunk should be readable");
    Point loaded;
    chunk.read(reinterpret_cast<char*>(&loaded), static_cast<std::streamsize>(sizeof(Point)));
    require(static_cast<bool>(chunk), "v3 RGB chunk should contain one complete point");

    require(loaded.has_color, "v3 binary point payload should preserve has_color");
    require(loaded.r == 37, "v3 binary point payload should preserve red channel");
    require(loaded.g == 216, "v3 binary point payload should preserve green channel");
    require(loaded.b == 101, "v3 binary point payload should preserve blue channel");

    std::filesystem::remove_all(dir);
}

void test_writer_refuses_existing_output_without_force() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_existing_output_test.termcloud";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    std::ofstream(dir / "sentinel.txt") << "keep";

    auto cloud = make_cloud(2);
    require_load_error_contains("already exists", [&]() {
        term_pcl::writeTermcloudV3Index(cloud, dir.string(), 3, 3);
    });
    require(std::filesystem::exists(dir / "sentinel.txt"), "refused write should not delete existing files");

    std::filesystem::remove_all(dir);
}

void test_writer_force_overwrites_existing_termcloud_directory() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_force_output_test.termcloud";
    std::filesystem::remove_all(dir);

    auto cloud = make_cloud(2);
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), 3, 3);
    std::ofstream(dir / "stale.txt") << "stale";

    term_pcl::TermcloudWriteOptions options;
    options.force = true;
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), options, 3, 3);

    require(std::filesystem::exists(dir / "metadata.txt"), "forced write should recreate metadata");
    require(!std::filesystem::exists(dir / "stale.txt"), "forced write should remove stale files");

    std::filesystem::remove_all(dir);
}

void test_v3_reader_rejects_truncated_metadata() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_v3_truncated_metadata_test.termcloud";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    std::ofstream(dir / "metadata.txt") << "termcloud 3\nsample.xyz\n10\n";
    std::ofstream(dir / "nodes.txt") << "";

    require_load_error_contains("Malformed termcloud metadata", [&]() {
        term_pcl::readTermcloudHierarchyMetadata(dir.string());
    });

    std::filesystem::remove_all(dir);
}

void test_v3_reader_rejects_duplicate_node_ids() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_v3_duplicate_node_test.termcloud";
    std::filesystem::remove_all(dir);

    auto cloud = make_cloud(4);
    term_pcl::TermcloudWriteOptions options;
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), options, 2, 2);

    std::ofstream nodes(dir / "nodes.txt");
    nodes << "0 0 0 2 0 1 0 1 0 0 chunks/000000.bin 0\n"
          << "0 1 2 2 2 3 0 1 0 0 chunks/000001.bin 0\n";
    nodes.close();

    require_load_error_contains("Malformed termcloud node metadata", [&]() {
        term_pcl::readTermcloudHierarchyMetadata(dir.string());
    });

    std::filesystem::remove_all(dir);
}

void test_v3_validation_rejects_truncated_chunk() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_v3_truncated_chunk_test.termcloud";
    std::filesystem::remove_all(dir);

    auto cloud = make_cloud(1);
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), 3, 3);
    const auto metadata = term_pcl::readTermcloudHierarchyMetadata(dir.string());
    std::ofstream(dir / metadata.index.nodes.front().chunk_file, std::ios::binary) << "x";

    require_load_error_contains("Malformed termcloud chunk", [&]() {
        term_pcl::validateTermcloudHierarchy(dir.string());
    });

    std::filesystem::remove_all(dir);
}

void test_v3_reader_rejects_child_cycle() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_v3_cycle_test.termcloud";
    std::filesystem::remove_all(dir);

    auto cloud = make_cloud(4);
    term_pcl::TermcloudWriteOptions options;
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), options, 2, 2);

    std::ofstream nodes(dir / "nodes.txt");
    nodes << "0 0 0 4 0 3 0 3 0 0 chunks/0.bin 1 1\n"
          << "1 1 0 4 0 3 0 3 0 0 chunks/1.bin 1 0\n";
    nodes.close();

    require_load_error_contains("Malformed termcloud node metadata", [&]() {
        term_pcl::readTermcloudHierarchyMetadata(dir.string());
    });

    std::filesystem::remove_all(dir);
}

void test_v3_reader_rejects_unreachable_node() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_v3_unreachable_test.termcloud";
    std::filesystem::remove_all(dir);

    auto cloud = make_cloud(4);
    term_pcl::TermcloudWriteOptions options;
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), options, 2, 2);

    std::ofstream nodes(dir / "nodes.txt");
    nodes << "0 0 0 2 0 1 0 1 0 0 chunks/0.bin 0\n"
          << "1 1 2 2 2 3 2 3 0 0 chunks/1.bin 0\n";
    nodes.close();

    require_load_error_contains("Malformed termcloud node metadata", [&]() {
        term_pcl::readTermcloudHierarchyMetadata(dir.string());
    });

    std::filesystem::remove_all(dir);
}

void test_v3_reader_rejects_child_range_outside_parent() {
    const auto dir = std::filesystem::temp_directory_path() / "termcloud_v3_bad_child_range_test.termcloud";
    std::filesystem::remove_all(dir);

    auto cloud = make_cloud(4);
    term_pcl::TermcloudWriteOptions options;
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), options, 2, 2);

    std::ofstream nodes(dir / "nodes.txt");
    nodes << "0 0 0 2 0 1 0 1 0 0 chunks/0.bin 1 1\n"
          << "1 1 2 2 2 3 2 3 0 0 chunks/1.bin 0\n";
    nodes.close();

    require_load_error_contains("Malformed termcloud node metadata", [&]() {
        term_pcl::readTermcloudHierarchyMetadata(dir.string());
    });

    std::filesystem::remove_all(dir);
}

}  // namespace

int main() {
    test_termcloud_v1_reader_compatibility();
    test_legacy_v1_xyz_only_points_bin_loads_uncolored_points();
    test_termcloud_v2_writes_chunk_directory();
    test_legacy_v2_xyz_only_chunks_load_uncolored_points();
    test_v3_writer_creates_hierarchy_metadata_and_node_chunks();
    test_termcloud_v3_reader_accepts_metadata_without_has_rgb();
    test_termcloud_v3_preserves_rgb_points();
    test_writer_refuses_existing_output_without_force();
    test_writer_force_overwrites_existing_termcloud_directory();
    test_v3_reader_rejects_truncated_metadata();
    test_v3_reader_rejects_duplicate_node_ids();
    test_v3_validation_rejects_truncated_chunk();
    test_v3_reader_rejects_child_cycle();
    test_v3_reader_rejects_unreachable_node();
    test_v3_reader_rejects_child_range_outside_parent();
    const std::filesystem::path output = "build/test-output/sample.termcloud";
    std::filesystem::remove_all(output);
    std::filesystem::create_directories(output.parent_path());

    const auto source = term_pcl::loadPointCloud("tests/fixtures/sample.xyz");
    term_pcl::writeTermcloudIndex(*source, output.string());
    const auto loaded = term_pcl::readTermcloudIndex(output.string());

    require(loaded->points.size() == source->points.size(), "termcloud should preserve point count");
    require(loaded->frame_id == source->frame_id, "termcloud should preserve frame id");
    require_close(loaded->min_x, source->min_x, "termcloud should preserve min x");
    require_close(loaded->max_x, source->max_x, "termcloud should preserve max x");
    require_close(loaded->points.front().x, source->points.front().x, "termcloud should preserve first point x");
    require_close(loaded->points.front().y, source->points.front().y, "termcloud should preserve first point y");
    require_close(loaded->points.front().z, source->points.front().z, "termcloud should preserve first point z");

    std::cout << "termcloud index tests passed\n";
    return 0;
}
