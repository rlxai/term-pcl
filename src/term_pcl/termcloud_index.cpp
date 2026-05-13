#include "term_pcl/termcloud_index.hpp"

#include "term_pcl/cloud_optimizer.hpp"
#include "term_pcl/point_cloud_loader.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace term_pcl {
namespace {

using term_pcl::CloudData;
using term_pcl::Point;

constexpr const char* kVersion = "termcloud 2";
constexpr const char* kVersion1 = "termcloud 1";
constexpr const char* kVersion3 = "termcloud 3";
constexpr std::size_t kChunkSize = 4096;
constexpr std::size_t kLegacyXyzPointSize = 3 * sizeof(float);

Point readLegacyXyzPoint(std::istream& input, const std::filesystem::path& path) {
    float xyz[3]{};
    input.read(reinterpret_cast<char*>(xyz), static_cast<std::streamsize>(sizeof(xyz)));
    if (!input) {
        throw LoadError("Malformed termcloud points: " + path.string());
    }
    return Point{xyz[0], xyz[1], xyz[2]};
}

std::filesystem::path metadataPath(const std::filesystem::path& root) {
    return root / "metadata.txt";
}

std::filesystem::path pointsPath(const std::filesystem::path& root) {
    return root / "points.bin";
}

std::filesystem::path chunksPath(const std::filesystem::path& root) {
    return root / "chunks";
}

void prepareOutputRoot(const std::filesystem::path& root, const TermcloudWriteOptions& options) {
    if (root.extension() != ".termcloud") {
        throw LoadError("Output path must end with .termcloud: " + root.string());
    }
    if (std::filesystem::is_symlink(root)) {
        throw LoadError("Refusing to write through symlink output path: " + root.string());
    }
    if (std::filesystem::exists(root)) {
        if (!options.force) {
            throw LoadError("Output path already exists: " + root.string() + " (use --force to overwrite)");
        }
        if (!std::filesystem::is_directory(root)) {
            throw LoadError("Output path already exists and is not a directory: " + root.string());
        }
        std::filesystem::remove_all(root);
    }
    std::filesystem::create_directories(root);
}

std::filesystem::path safeChunkPath(const std::filesystem::path& root, const std::string& chunk_file) {
    const std::filesystem::path relative(chunk_file);
    if (relative.empty() || relative.is_absolute()) {
        throw LoadError("Malformed termcloud node metadata: " + root.string());
    }
    for (const auto& part : relative) {
        if (part == "..") {
            throw LoadError("Malformed termcloud node metadata: " + root.string());
        }
    }
    return root / relative;
}

void validateChunkFile(const std::filesystem::path& chunk_path, std::size_t point_count) {
    std::ifstream input(chunk_path, std::ios::binary | std::ios::ate);
    if (!input) {
        throw LoadError("Failed to read termcloud chunk: " + chunk_path.string());
    }
    const auto size = input.tellg();
    if (size < 0) {
        throw LoadError("Malformed termcloud chunk: " + chunk_path.string());
    }
    const auto byte_count = static_cast<std::size_t>(size);
    if (byte_count != point_count * sizeof(Point)) {
        throw LoadError("Malformed termcloud chunk: " + chunk_path.string());
    }
}

void validateHierarchyGraph(const HierarchicalLodIndex& index, const std::string& path) {
    if (index.nodes.empty()) {
        if (index.point_count != 0) {
            throw LoadError("Malformed termcloud node metadata: " + path);
        }
        return;
    }
    if (index.root_node >= index.nodes.size()) {
        throw LoadError("Malformed termcloud node metadata: " + path);
    }

    std::vector<unsigned char> state(index.nodes.size(), 0);
    std::vector<std::size_t> parent(index.nodes.size(), index.nodes.size());
    std::size_t visited_count = 0;

    const std::function<void(std::size_t)> visit = [&](std::size_t node_id) {
        if (state[node_id] == 1) {
            throw LoadError("Malformed termcloud node metadata: " + path);
        }
        if (state[node_id] == 2) return;
        state[node_id] = 1;
        ++visited_count;
        const auto& node = index.nodes[node_id];
        for (const auto child_id : node.children) {
            const auto& child = index.nodes[child_id];
            if (parent[child_id] != index.nodes.size() || child.level != node.level + 1 ||
                child.first_point < node.first_point ||
                child.first_point + child.point_count > node.first_point + node.point_count) {
                throw LoadError("Malformed termcloud node metadata: " + path);
            }
            parent[child_id] = node_id;
            visit(child_id);
        }
        state[node_id] = 2;
    };

    visit(index.root_node);
    if (visited_count != index.nodes.size()) {
        throw LoadError("Malformed termcloud node metadata: " + path);
    }
}

void readPointsBin(const std::filesystem::path& points_path, CloudData& cloud, std::size_t point_count) {
    std::ifstream points(points_path, std::ios::binary | std::ios::ate);
    if (!points) {
        throw LoadError("Failed to read termcloud points: " + points_path.string());
    }
    const auto size = points.tellg();
    if (size < 0) {
        throw LoadError("Malformed termcloud points: " + points_path.string());
    }
    points.seekg(0);

    const auto byte_count = static_cast<std::size_t>(size);
    if (byte_count == point_count * sizeof(Point)) {
        cloud.points.resize(point_count);
        points.read(reinterpret_cast<char*>(cloud.points.data()),
                    static_cast<std::streamsize>(cloud.points.size() * sizeof(Point)));
        if (!points) {
            throw LoadError("Malformed termcloud points: " + points_path.string());
        }
        return;
    }

    if (byte_count == point_count * kLegacyXyzPointSize) {
        cloud.points.clear();
        cloud.points.reserve(point_count);
        for (std::size_t i = 0; i < point_count; ++i) {
            cloud.points.push_back(readLegacyXyzPoint(points, points_path));
        }
        return;
    }

    throw LoadError("Malformed termcloud points: " + points_path.string());
}

void readChunkFiles(const std::filesystem::path& chunks_path, CloudData& cloud, std::size_t point_count) {
    if (!std::filesystem::is_directory(chunks_path)) {
        throw LoadError("Failed to read termcloud chunks: " + chunks_path.string());
    }

    std::vector<std::filesystem::path> chunk_files;
    for (const auto& entry : std::filesystem::directory_iterator(chunks_path)) {
        if (entry.is_regular_file()) {
            chunk_files.push_back(entry.path());
        }
    }
    std::sort(chunk_files.begin(), chunk_files.end());

    cloud.points.clear();
    cloud.points.reserve(point_count);
    for (const auto& chunk_file : chunk_files) {
        std::ifstream input(chunk_file, std::ios::binary | std::ios::ate);
        if (!input) {
            throw LoadError("Failed to read termcloud chunk: " + chunk_file.string());
        }
        const auto size = input.tellg();
        if (size < 0) {
            throw LoadError("Malformed termcloud chunk: " + chunk_file.string());
        }
        input.seekg(0);
        const auto byte_count = static_cast<std::size_t>(size);
        if (byte_count % sizeof(Point) == 0) {
            const auto count = byte_count / sizeof(Point);
            const auto offset = cloud.points.size();
            cloud.points.resize(offset + count);
            input.read(reinterpret_cast<char*>(cloud.points.data() + offset), static_cast<std::streamsize>(count * sizeof(Point)));
            if (!input) {
                throw LoadError("Malformed termcloud chunk: " + chunk_file.string());
            }
        } else if (byte_count % kLegacyXyzPointSize == 0) {
            const auto count = byte_count / kLegacyXyzPointSize;
            cloud.points.reserve(cloud.points.size() + count);
            for (std::size_t i = 0; i < count; ++i) {
                cloud.points.push_back(readLegacyXyzPoint(input, chunk_file));
            }
        } else {
            throw LoadError("Malformed termcloud chunk: " + chunk_file.string());
        }
    }

    if (cloud.points.size() != point_count) {
        throw LoadError("Malformed termcloud chunks: expected " + std::to_string(point_count) +
                        " points, got " + std::to_string(cloud.points.size()));
    }
}

}  // namespace

void writeTermcloudIndex(const CloudData& cloud,
                         const std::string& output_path,
                         const TermcloudWriteOptions& options) {
    const std::filesystem::path root(output_path);
    prepareOutputRoot(root, options);
    std::filesystem::create_directories(chunksPath(root));

    std::ofstream metadata(metadataPath(root));
    if (!metadata) {
        throw LoadError("Failed to write termcloud metadata: " + output_path);
    }

    const auto chunk_count = (cloud.points.size() + kChunkSize - 1) / kChunkSize;
    metadata << kVersion << '\n'
             << cloud.frame_id << '\n'
             << cloud.points.size() << '\n'
             << cloud.min_x << ' ' << cloud.max_x << '\n'
             << cloud.min_y << ' ' << cloud.max_y << '\n'
             << cloud.min_z << ' ' << cloud.max_z << '\n'
             << cloud.cx << ' ' << cloud.cy << ' ' << cloud.cz << '\n'
             << "chunk_size " << kChunkSize << '\n'
             << "chunk_count " << chunk_count << '\n';
    if (!metadata) {
        throw LoadError("Failed to write termcloud metadata: " + output_path);
    }

    for (std::size_t first = 0, chunk_index = 0; first < cloud.points.size(); first += kChunkSize, ++chunk_index) {
        const auto count = std::min(kChunkSize, cloud.points.size() - first);
        std::ostringstream name;
        name << std::setw(6) << std::setfill('0') << chunk_index << ".bin";
        const auto chunk_path = chunksPath(root) / name.str();
        std::ofstream output(chunk_path, std::ios::binary);
        if (!output) {
            throw LoadError("Failed to write termcloud chunk: " + chunk_path.string());
        }
        output.write(reinterpret_cast<const char*>(cloud.points.data() + first),
                     static_cast<std::streamsize>(count * sizeof(Point)));
        if (!output) {
            throw LoadError("Failed to write termcloud chunk: " + chunk_path.string());
        }
    }
}

void writeTermcloudV3Index(const CloudData& cloud,
                           const std::string& output_path,
                           std::size_t leaf_point_limit,
                           std::size_t max_depth) {
    writeTermcloudV3Index(cloud, output_path, TermcloudWriteOptions{}, leaf_point_limit, max_depth);
}

void writeTermcloudV3Index(const CloudData& cloud,
                           const std::string& output_path,
                           const TermcloudWriteOptions& options,
                           std::size_t leaf_point_limit,
                           std::size_t max_depth) {
    const std::filesystem::path root(output_path);
    prepareOutputRoot(root, options);
    std::filesystem::create_directories(chunksPath(root));

    auto index = buildHierarchicalLodIndex(cloud, leaf_point_limit, max_depth);
    const bool has_rgb = std::any_of(cloud.points.begin(), cloud.points.end(), [](const Point& point) {
        return point.has_color;
    });

    std::ofstream metadata(metadataPath(root));
    if (!metadata) {
        throw LoadError("Failed to write termcloud metadata: " + output_path);
    }
    metadata << kVersion3 << '\n'
             << index.frame_id << '\n'
             << index.point_count << '\n'
             << index.min_x << ' ' << index.max_x << '\n'
             << index.min_y << ' ' << index.max_y << '\n'
             << index.min_z << ' ' << index.max_z << '\n'
             << index.cx << ' ' << index.cy << ' ' << index.cz << '\n'
             << index.nodes.size() << '\n'
             << "has_rgb " << (has_rgb ? 1 : 0) << '\n';
    if (!metadata) {
        throw LoadError("Failed to write termcloud metadata: " + output_path);
    }

    std::ofstream nodes(root / "nodes.txt");
    if (!nodes) {
        throw LoadError("Failed to write termcloud nodes: " + output_path);
    }
    for (const auto& node : index.nodes) {
        nodes << node.id << ' ' << node.level << ' ' << node.first_point << ' ' << node.point_count << ' '
              << node.min_x << ' ' << node.max_x << ' '
              << node.min_y << ' ' << node.max_y << ' '
              << node.min_z << ' ' << node.max_z << ' '
              << node.chunk_file << ' ' << node.children.size();
        for (const auto child_id : node.children) {
            nodes << ' ' << child_id;
        }
        nodes << '\n';

        const auto chunk_path = root / node.chunk_file;
        std::filesystem::create_directories(chunk_path.parent_path());
        std::ofstream output(chunk_path, std::ios::binary);
        if (!output) {
            throw LoadError("Failed to write termcloud chunk: " + chunk_path.string());
        }
        output.write(reinterpret_cast<const char*>(cloud.points.data() + node.first_point),
                     static_cast<std::streamsize>(node.point_count * sizeof(Point)));
        if (!output) {
            throw LoadError("Failed to write termcloud chunk: " + chunk_path.string());
        }
    }
    if (!nodes) {
        throw LoadError("Failed to write termcloud nodes: " + output_path);
    }
}

TermcloudHierarchyMetadata readTermcloudHierarchyMetadata(const std::string& path) {
    const std::filesystem::path root(path);
    std::ifstream metadata(metadataPath(root));
    if (!metadata) {
        throw LoadError("Failed to read termcloud metadata: " + path);
    }

    std::string version;
    std::getline(metadata, version);
    if (version != kVersion3) {
        throw LoadError("Unsupported termcloud hierarchy version: " + version);
    }

    TermcloudHierarchyMetadata result;
    result.base_path = path;
    auto& index = result.index;
    std::getline(metadata, index.frame_id);
    std::size_t node_count = 0;
    metadata >> index.point_count;
    metadata >> index.min_x >> index.max_x;
    metadata >> index.min_y >> index.max_y;
    metadata >> index.min_z >> index.max_z;
    metadata >> index.cx >> index.cy >> index.cz;
    metadata >> node_count;
    if (!metadata) {
        throw LoadError("Malformed termcloud metadata: " + path);
    }

    metadata >> std::ws;
    if (metadata.peek() != std::char_traits<char>::eof()) {
        std::string key;
        int value = 0;
        metadata >> key >> value;
        if (!metadata || key != "has_rgb" || (value != 0 && value != 1)) {
            throw LoadError("Malformed termcloud metadata: " + path);
        }
        result.has_rgb = value == 1;
    }

    std::ifstream nodes(root / "nodes.txt");
    if (!nodes) {
        throw LoadError("Failed to read termcloud nodes: " + path);
    }
    index.nodes.resize(node_count);
    std::vector<bool> seen(node_count, false);
    for (std::size_t i = 0; i < node_count; ++i) {
        LodNode node;
        std::size_t child_count = 0;
        nodes >> node.id >> node.level >> node.first_point >> node.point_count
              >> node.min_x >> node.max_x
              >> node.min_y >> node.max_y
              >> node.min_z >> node.max_z
              >> node.chunk_file >> child_count;
        if (!nodes) {
            throw LoadError("Malformed termcloud node metadata: " + path);
        }
        node.children.resize(child_count);
        for (std::size_t child = 0; child < child_count; ++child) {
            nodes >> node.children[child];
        }
        if (!nodes || node.id >= index.nodes.size() || seen[node.id] ||
            node.first_point > index.point_count || node.point_count > index.point_count - node.first_point) {
            throw LoadError("Malformed termcloud node metadata: " + path);
        }
        safeChunkPath(root, node.chunk_file);
        for (const auto child_id : node.children) {
            if (child_id >= node_count || child_id == node.id) {
                throw LoadError("Malformed termcloud node metadata: " + path);
            }
        }
        seen[node.id] = true;
        index.nodes[node.id] = std::move(node);
    }
    nodes >> std::ws;
    if (nodes.peek() != std::char_traits<char>::eof()) {
        throw LoadError("Malformed termcloud node metadata: " + path);
    }
    if (std::find(seen.begin(), seen.end(), false) != seen.end()) {
        throw LoadError("Malformed termcloud node metadata: " + path);
    }
    index.root_node = index.nodes.empty() ? 0 : 0;
    validateHierarchyGraph(index, path);
    return result;
}

void validateTermcloudHierarchy(const std::string& path) {
    const std::filesystem::path root(path);
    const auto metadata = readTermcloudHierarchyMetadata(path);
    for (const auto& node : metadata.index.nodes) {
        validateChunkFile(safeChunkPath(root, node.chunk_file), node.point_count);
    }
}

std::shared_ptr<CloudData> readTermcloudIndex(const std::string& input_path) {
    const std::filesystem::path root(input_path);
    std::ifstream metadata(metadataPath(root));
    if (!metadata) {
        throw LoadError("Failed to read termcloud metadata: " + input_path);
    }

    std::string version;
    std::getline(metadata, version);
    if (version != kVersion && version != kVersion1 && version != kVersion3) {
        throw LoadError("Unsupported termcloud version: " + version);
    }

    if (version == kVersion3) {
        const auto hierarchy = readTermcloudHierarchyMetadata(input_path);
        auto cloud = std::make_shared<CloudData>();
        cloud->frame_id = hierarchy.index.frame_id;
        cloud->min_x = hierarchy.index.min_x;
        cloud->max_x = hierarchy.index.max_x;
        cloud->min_y = hierarchy.index.min_y;
        cloud->max_y = hierarchy.index.max_y;
        cloud->min_z = hierarchy.index.min_z;
        cloud->max_z = hierarchy.index.max_z;
        cloud->cx = hierarchy.index.cx;
        cloud->cy = hierarchy.index.cy;
        cloud->cz = hierarchy.index.cz;
        return cloud;
    }

    auto cloud = std::make_shared<CloudData>();
    std::getline(metadata, cloud->frame_id);
    std::size_t point_count = 0;
    metadata >> point_count;
    metadata >> cloud->min_x >> cloud->max_x;
    metadata >> cloud->min_y >> cloud->max_y;
    metadata >> cloud->min_z >> cloud->max_z;
    metadata >> cloud->cx >> cloud->cy >> cloud->cz;
    if (!metadata) {
        throw LoadError("Malformed termcloud metadata: " + input_path);
    }

    if (version == kVersion1) {
        readPointsBin(pointsPath(root), *cloud, point_count);
    } else {
        readChunkFiles(chunksPath(root), *cloud, point_count);
    }

    return cloud;
}

}  // namespace term_pcl
