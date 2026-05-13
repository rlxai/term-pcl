#include "term_pcl/cloud_optimizer.hpp"
#include "term_pcl/point_cloud_loader.hpp"
#include "term_pcl/termcloud_index.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_close(float actual, float expected, const std::string& message) {
    require(std::fabs(actual - expected) < 0.0001f, message);
}

void test_xyzrgb_preserves_rgb_columns() {
    const std::string path = "build/test-output/rgb.xyzrgb";
    std::filesystem::create_directories("build/test-output");
    {
        std::ofstream out(path);
        out << "1 2 3 37 216 101\n";
    }

    const auto cloud = term_pcl::loadPointCloud(path);
    require(cloud->points.size() == 1, "xyzrgb should load one point");
    require(cloud->points[0].has_color, "xyzrgb should mark point as colored");
    require(cloud->points[0].r == 37 && cloud->points[0].g == 216 && cloud->points[0].b == 101,
            "xyzrgb should preserve RGB values");
}

void test_ply_preserves_rgb_properties() {
    const std::string path = "build/test-output/rgb.ply";
    std::filesystem::create_directories("build/test-output");
    {
        std::ofstream out(path);
        out << "ply\nformat ascii 1.0\nelement vertex 1\n"
            << "property float x\nproperty float y\nproperty float z\n"
            << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            << "end_header\n1 2 3 37 216 101\n";
    }

    const auto cloud = term_pcl::loadPointCloud(path);
    require(cloud->points.size() == 1, "PLY should load one point");
    require(cloud->points[0].has_color, "PLY should mark RGB point as colored");
    require(cloud->points[0].r == 37 && cloud->points[0].g == 216 && cloud->points[0].b == 101,
            "PLY should preserve RGB values");
}

void test_ply_preserves_black_rgb_properties() {
    const std::string path = "build/test-output/black_rgb.ply";
    std::filesystem::create_directories("build/test-output");
    {
        std::ofstream out(path);
        out << "ply\nformat ascii 1.0\nelement vertex 1\n"
            << "property float x\nproperty float y\nproperty float z\n"
            << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            << "end_header\n1 2 3 0 0 0\n";
    }

    const auto cloud = term_pcl::loadPointCloud(path);
    require(cloud->points.size() == 1, "black RGB PLY should load one point");
    require(cloud->points[0].has_color, "black RGB PLY should mark RGB point as colored");
    require(cloud->points[0].r == 0 && cloud->points[0].g == 0 && cloud->points[0].b == 0,
            "black RGB PLY should preserve zero RGB values");
}

void test_voxel_filter_preserves_xyzrgb_color() {
    const std::string path = "build/test-output/voxel_rgb.xyzrgb";
    std::filesystem::create_directories("build/test-output");
    {
        std::ofstream out(path);
        out << "0.00 0.00 0.00 250 10 20\n"
            << "0.01 0.01 0.01 10 240 30\n"
            << "1.00 1.00 1.00 40 50 220\n";
    }

    term_pcl::LoadOptions options;
    options.voxel_size = 0.5f;
    const auto cloud = term_pcl::loadPointCloud(path, options);
    require(cloud->points.size() == 2, "voxel filter should collapse nearby xyzrgb points");

    bool found_colored_non_default_point = false;
    const std::vector<std::vector<unsigned char>> input_colors = {
        {250, 10, 20}, {10, 240, 30}, {40, 50, 220}
    };
    for (const auto& point : cloud->points) {
        require(point.has_color, "voxel-filtered xyzrgb points should stay marked as colored");
        if (point.r != 255 || point.g != 255 || point.b != 255) {
            found_colored_non_default_point = true;
        }
        bool could_match_or_average_input_color = false;
        for (const auto& color : input_colors) {
            if (std::abs(static_cast<int>(point.r) - static_cast<int>(color[0])) <= 120 &&
                std::abs(static_cast<int>(point.g) - static_cast<int>(color[1])) <= 120 &&
                std::abs(static_cast<int>(point.b) - static_cast<int>(color[2])) <= 120) {
                could_match_or_average_input_color = true;
            }
        }
        require(could_match_or_average_input_color,
                "voxel-filtered xyzrgb color should preserve or average input RGB values");
    }
    require(found_colored_non_default_point,
            "voxel-filtered xyzrgb output should keep non-default RGB from input");
}

void test_voxel_filter_keeps_uncolored_voxels_uncolored_when_cloud_has_some_rgb() {
    const std::string uncolored_path = "build/test-output/mixed_uncolored.xyz";
    const std::string colored_path = "build/test-output/mixed_colored.xyzrgb";
    const std::string termcloud_path = "build/test-output/mixed_optional_color.termcloud";
    std::filesystem::create_directories("build/test-output");
    std::filesystem::remove_all(termcloud_path);
    {
        std::ofstream out(uncolored_path);
        out << "0.00 0.00 0.00\n"
            << "0.01 0.01 0.01\n";
    }
    {
        std::ofstream out(colored_path);
        out << "2.00 2.00 2.00 37 216 101\n";
    }

    auto uncolored = term_pcl::loadPointCloud(uncolored_path);
    auto colored = term_pcl::loadPointCloud(colored_path);
    uncolored->points.insert(uncolored->points.end(), colored->points.begin(), colored->points.end());
    term_pcl::recomputeStats(*uncolored);
    term_pcl::writeTermcloudIndex(*uncolored, termcloud_path);

    term_pcl::LoadOptions options;
    options.voxel_size = 0.5f;
    const auto cloud = term_pcl::loadPointCloud(termcloud_path, options);
    require(cloud->points.size() == 2, "voxel filter should keep separate colored and uncolored voxels");

    bool found_uncolored_voxel = false;
    for (const auto& point : cloud->points) {
        if (point.x < 1.0f) {
            found_uncolored_voxel = true;
            require(!point.has_color, "voxel from only uncolored points should remain uncolored in mixed cloud");
        }
    }
    require(found_uncolored_voxel, "mixed cloud should include the uncolored voxel");
}

}  // namespace

int main() {
    test_xyzrgb_preserves_rgb_columns();
    test_ply_preserves_rgb_properties();
    test_ply_preserves_black_rgb_properties();
    test_voxel_filter_preserves_xyzrgb_color();
    test_voxel_filter_keeps_uncolored_voxels_uncolored_when_cloud_has_some_rgb();

    const auto xyz = term_pcl::loadPointCloud("tests/fixtures/sample.xyz");
    require(xyz->points.size() == 3, "xyz loader should read three points");
    require_close(xyz->points[1].x, 1.0f, "xyz loader should preserve x");
    require_close(xyz->points[1].y, 2.0f, "xyz loader should preserve y");
    require_close(xyz->points[1].z, 3.0f, "xyz loader should preserve z");
    require_close(xyz->min_x, -1.0f, "xyz loader should compute min x");
    require_close(xyz->max_x, 1.0f, "xyz loader should compute max x");
    require_close(xyz->cx, 0.0f, "xyz loader should compute center x from bounds");
    require_close(xyz->cy, 1.0f, "xyz loader should compute center y from bounds");
    require_close(xyz->cz, 1.5f, "xyz loader should compute center z from bounds");

    const auto xyzn = term_pcl::loadPointCloud("tests/fixtures/sample.xyzn");
    require(xyzn->points.size() == 2, "xyzn loader should validate normals and read coordinates");

    const auto xyzrgb = term_pcl::loadPointCloud("tests/fixtures/sample.xyzrgb");
    require(xyzrgb->points.size() == 2, "xyzrgb loader should validate colors and read coordinates");

    const auto pcd = term_pcl::loadPointCloud("tests/fixtures/sample.pcd");
    require(pcd->points.size() == 3, "pcd loader should read three points");

    try {
        (void)term_pcl::loadPointCloud("tests/fixtures/sample.unsupported");
        throw std::runtime_error("unsupported extension should throw LoadError");
    } catch (const term_pcl::LoadError& error) {
        require(std::string(error.what()).find("Unsupported point cloud format") != std::string::npos,
                "unsupported extension should explain supported formats");
    }

    try {
        (void)term_pcl::loadPointCloud("tests/fixtures/bad.xyz");
        throw std::runtime_error("malformed xyz should throw LoadError");
    } catch (const term_pcl::LoadError& error) {
        require(std::string(error.what()).find("Malformed point") != std::string::npos,
                "malformed xyz should identify malformed point data");
    }

    try {
        (void)term_pcl::loadPointCloud("tests/fixtures/bad.xyzn");
        throw std::runtime_error("malformed xyzn should throw LoadError");
    } catch (const term_pcl::LoadError& error) {
        require(std::string(error.what()).find("Malformed point") != std::string::npos,
                "malformed xyzn should identify malformed point data");
    }

    try {
        (void)term_pcl::loadPointCloud("tests/fixtures/bad.xyzrgb");
        throw std::runtime_error("malformed xyzrgb should throw LoadError");
    } catch (const term_pcl::LoadError& error) {
        require(std::string(error.what()).find("Malformed point") != std::string::npos,
                "malformed xyzrgb should identify malformed point data");
    }

    std::cout << "point cloud loader tests passed\n";
    return 0;
}
