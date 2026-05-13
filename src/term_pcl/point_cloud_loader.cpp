#include "term_pcl/point_cloud_loader.hpp"

#include "term_pcl/cloud_optimizer.hpp"
#include "term_pcl/termcloud_index.hpp"

#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace term_pcl {
namespace {

using term_pcl::CloudData;
using term_pcl::Point;

std::string lowercaseExtension(const std::filesystem::path& path) {
    auto extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return extension;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPclCloud(const std::vector<Point>& points) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(points.size());
    for (const auto& point : points) {
        cloud->push_back(pcl::PointXYZ{point.x, point.y, point.z});
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr toPclRgbCloud(const std::vector<Point>& points) {
    auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud->reserve(points.size());
    for (const auto& point : points) {
        pcl::PointXYZRGB output;
        output.x = point.x;
        output.y = point.y;
        output.z = point.z;
        output.r = point.r;
        output.g = point.g;
        output.b = point.b;
        cloud->push_back(output);
    }
    return cloud;
}

std::vector<Point> fromPclCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    std::vector<Point> points;
    points.reserve(cloud.size());
    for (const auto& point : cloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            points.push_back(Point{point.x, point.y, point.z});
        }
    }
    return points;
}

std::vector<Point> fromPclRgbCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    std::vector<Point> points;
    points.reserve(cloud.size());
    for (const auto& point : cloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            Point output;
            output.x = point.x;
            output.y = point.y;
            output.z = point.z;
            output.r = point.r;
            output.g = point.g;
            output.b = point.b;
            output.has_color = true;
            points.push_back(output);
        }
    }
    return points;
}

bool pcdHeaderHasRgbFields(std::istream& input) {
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream stream(line);
        std::string keyword;
        stream >> keyword;
        std::transform(keyword.begin(), keyword.end(), keyword.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        if (keyword == "fields") {
            std::string field;
            while (stream >> field) {
                std::transform(field.begin(), field.end(), field.begin(), [](unsigned char ch) {
                    return static_cast<char>(std::tolower(ch));
                });
                if (field == "rgb" || field == "rgba" || field == "red" || field == "green" || field == "blue") {
                    return true;
                }
            }
        }
        if (keyword == "data") {
            break;
        }
    }
    return false;
}

bool plyHeaderHasRgbFields(std::istream& input) {
    bool in_vertex_element = false;
    bool has_packed_rgb = false;
    bool has_red = false;
    bool has_green = false;
    bool has_blue = false;
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream stream(line);
        std::string keyword;
        stream >> keyword;
        std::transform(keyword.begin(), keyword.end(), keyword.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });

        if (keyword == "element") {
            std::string element_name;
            stream >> element_name;
            std::transform(element_name.begin(), element_name.end(), element_name.begin(), [](unsigned char ch) {
                return static_cast<char>(std::tolower(ch));
            });
            in_vertex_element = element_name == "vertex";
            continue;
        }

        if (keyword == "property" && in_vertex_element) {
            std::vector<std::string> tokens;
            std::string token;
            while (stream >> token) {
                std::transform(token.begin(), token.end(), token.begin(), [](unsigned char ch) {
                    return static_cast<char>(std::tolower(ch));
                });
                tokens.push_back(token);
            }
            if (!tokens.empty()) {
                const auto& property_name = tokens.back();
                has_packed_rgb = has_packed_rgb || property_name == "rgb" || property_name == "rgba";
                has_red = has_red || property_name == "red" || property_name == "r";
                has_green = has_green || property_name == "green" || property_name == "g";
                has_blue = has_blue || property_name == "blue" || property_name == "b";
            }
        }

        if (keyword == "end_header") {
            break;
        }
    }
    return has_packed_rgb || (has_red && has_green && has_blue);
}

bool fileHasRgbFields(const std::filesystem::path& path, const std::string& extension) {
    std::ifstream input(path);
    if (!input) {
        return false;
    }
    if (extension == ".pcd") {
        return pcdHeaderHasRgbFields(input);
    }
    if (extension == ".ply") {
        return plyHeaderHasRgbFields(input);
    }
    return false;
}

class ScopedPclVerbosity {
public:
    ScopedPclVerbosity() : previous_(pcl::console::getVerbosityLevel()) {
        pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    }

    ~ScopedPclVerbosity() {
        pcl::console::setVerbosityLevel(previous_);
    }

private:
    pcl::console::VERBOSITY_LEVEL previous_;
};

std::vector<Point> applyVoxelFilter(std::vector<Point> points, float voxel_size) {
    if (voxel_size <= 0.0f || points.empty()) {
        return points;
    }

    const bool all_points_have_color = std::all_of(points.begin(), points.end(), [](const Point& point) {
        return point.has_color;
    });
    if (all_points_have_color) {
        auto input = toPclRgbCloud(points);
        pcl::PointCloud<pcl::PointXYZRGB> output;
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setInputCloud(input);
        filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        filter.filter(output);
        return fromPclRgbCloud(output);
    }

    auto input = toPclCloud(points);
    pcl::PointCloud<pcl::PointXYZ> output;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    filter.filter(output);
    return fromPclCloud(output);
}

std::shared_ptr<CloudData> makeCloudData(std::vector<Point> points, std::string frame_id, const LoadOptions& options) {
    points = applyVoxelFilter(std::move(points), options.voxel_size);
    if (points.empty()) {
        throw LoadError("Point cloud contains no points: " + frame_id);
    }

    auto data = std::make_shared<CloudData>();
    data->points = std::move(points);
    data->frame_id = std::move(frame_id);
    recomputeStats(*data);
    applyPointBudget(*data, options.point_budget);
    return data;
}

std::shared_ptr<CloudData> loadPclCloud(const std::filesystem::path& path, const std::string& extension, const LoadOptions& options) {
    if (extension == ".pcd" || extension == ".ply") {
        pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
        int rgb_result = -1;
        {
            ScopedPclVerbosity quiet_pcl;
            if (extension == ".pcd") {
                rgb_result = pcl::io::loadPCDFile(path.string(), rgb_cloud);
            } else if (extension == ".ply") {
                rgb_result = pcl::io::loadPLYFile(path.string(), rgb_cloud);
            }
        }
        if (rgb_result >= 0 && fileHasRgbFields(path, extension)) {
            auto rgb_points = fromPclRgbCloud(rgb_cloud);
            return makeCloudData(std::move(rgb_points), path.filename().string(), options);
        }
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    int result = -1;
    if (extension == ".pcd") {
        result = pcl::io::loadPCDFile(path.string(), cloud);
    } else if (extension == ".ply") {
        result = pcl::io::loadPLYFile(path.string(), cloud);
    }

    if (result < 0) {
        throw LoadError("Failed to load point cloud file: " + path.string());
    }

    return makeCloudData(fromPclCloud(cloud), path.filename().string(), options);
}

std::shared_ptr<CloudData> loadTextCloud(const std::filesystem::path& path, int expected_columns, bool has_rgb, const LoadOptions& options) {
    std::ifstream input(path);
    if (!input) {
        throw LoadError("Failed to open point cloud file: " + path.string());
    }

    std::vector<Point> points;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream stream(line);
        std::vector<float> values;
        float value = 0.0f;
        while (stream >> value) {
            values.push_back(value);
        }

        if (!stream.eof() || static_cast<int>(values.size()) != expected_columns) {
            throw LoadError("Malformed point at line " + std::to_string(line_number) + " in " + path.string());
        }
        Point point{values[0], values[1], values[2]};
        if (has_rgb) {
            point.r = static_cast<unsigned char>(std::clamp(values[3], 0.0f, 255.0f));
            point.g = static_cast<unsigned char>(std::clamp(values[4], 0.0f, 255.0f));
            point.b = static_cast<unsigned char>(std::clamp(values[5], 0.0f, 255.0f));
            point.has_color = true;
        }
        points.push_back(point);
    }

    return makeCloudData(std::move(points), path.filename().string(), options);
}

}  // namespace

LoadError::LoadError(const std::string& message) : std::runtime_error(message) {}

bool isTermcloudV3(const std::string& path_string) {
    const std::filesystem::path path(path_string);
    if (lowercaseExtension(path) != ".termcloud") return false;
    std::ifstream metadata(path / "metadata.txt");
    std::string version;
    std::getline(metadata, version);
    return version == "termcloud 3";
}

std::shared_ptr<CloudData> loadPointCloud(const std::string& path_string, const LoadOptions& options) {
    const std::filesystem::path path(path_string);
    const auto extension = lowercaseExtension(path);

    if (extension == ".termcloud") {
        auto cloud = readTermcloudIndex(path.string());
        if (!cloud->points.empty()) {
            cloud->points = applyVoxelFilter(std::move(cloud->points), options.voxel_size);
            recomputeStats(*cloud);
            applyPointBudget(*cloud, options.point_budget);
        }
        return cloud;
    }

    if (extension == ".pcd" || extension == ".ply") {
        return loadPclCloud(path, extension, options);
    }

    if (extension == ".xyz") {
        return loadTextCloud(path, 3, false, options);
    }

    if (extension == ".xyzn") {
        return loadTextCloud(path, 6, false, options);
    }

    if (extension == ".xyzrgb") {
        return loadTextCloud(path, 6, true, options);
    }

    throw LoadError("Unsupported point cloud format '" + extension + "'. Supported formats: .pcd, .ply, .xyz, .xyzn, .xyzrgb");
}

}  // namespace term_pcl
