#include "term_pcl/hierarchical_lod.hpp"
#include "term_pcl/lod_cloud_provider.hpp"
#include "term_pcl/point_cloud_loader.hpp"
#include "term_pcl/termcloud_index.hpp"
#include "term_pcl/cloud_provider.hpp"
#include "term_pcl/color_map.hpp"
#include "term_pcl/visualizer.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

struct CommandLine {
    bool check_only = false;
    bool index_mode = false;
    bool force = false;
    std::string path;
    std::string output_path;
    term_pcl::LoadOptions load_options;
    term_pcl::ColorSettings color_settings;
};

void printUsage(std::ostream& stream) {
    stream << "Usage: term-pcl [options] <point-cloud-file>\n"
           << "       term-pcl index <input-cloud> --output <output.termcloud> [options]\n"
           << "\n"
           << "Supported formats: .pcd, .ply, .xyz, .xyzn, .xyzrgb\n"
           << "\n"
           << "Options:\n"
           << "  --check             Load the file and print a summary without opening the terminal UI\n"
           << "  --point-budget N    Keep at most N loaded/rendered points\n"
           << "  --voxel-size M      Downsample with an M-unit voxel grid before rendering\n"
           << "  --color MODE         Color mode: rgb, elevation, x, white, rainbow, turbo, viridis, heat, grayscale\n"
           << "  --profile           Print load timing to stderr\n"
           << "  --output PATH       Output directory for index subcommand\n"
           << "  --force             Overwrite an existing .termcloud output directory when indexing\n"
           << "  -h, --help          Show this help text\n";
}

std::size_t parseSize(const std::string& value, const std::string& option) {
    try {
        std::size_t parsed_chars = 0;
        const auto parsed = std::stoull(value, &parsed_chars);
        if (parsed_chars != value.size() || parsed == 0) {
            throw std::invalid_argument("invalid");
        }
        return static_cast<std::size_t>(parsed);
    } catch (const std::exception&) {
        throw std::invalid_argument("Invalid " + option + ": " + value);
    }
}

float parseFloat(const std::string& value, const std::string& option) {
    try {
        std::size_t parsed_chars = 0;
        const auto parsed = std::stof(value, &parsed_chars);
        if (parsed_chars != value.size() || parsed <= 0.0f) {
            throw std::invalid_argument("invalid");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::invalid_argument("Invalid " + option + ": " + value);
    }
}

CommandLine parseCommandLine(int argc, char** argv) {
    CommandLine command;
    int start_index = 1;
    if (argc > 1 && std::string(argv[1]) == "index") {
        command.index_mode = true;
        start_index = 2;
    }

    for (int i = start_index; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--check") {
            command.check_only = true;
        } else if (arg == "--profile") {
            command.load_options.profile = true;
        } else if (arg == "--force") {
            command.force = true;
        } else if (arg == "--output") {
            if (++i >= argc) {
                throw std::invalid_argument("Missing --output value");
            }
            command.output_path = argv[i];
        } else if (arg == "--point-budget") {
            if (++i >= argc) {
                throw std::invalid_argument("Invalid --point-budget: missing value");
            }
            command.load_options.point_budget = parseSize(argv[i], "--point-budget");
        } else if (arg == "--voxel-size") {
            if (++i >= argc) {
                throw std::invalid_argument("Invalid --voxel-size: missing value");
            }
            command.load_options.voxel_size = parseFloat(argv[i], "--voxel-size");
        } else if (arg == "--color") {
            if (++i >= argc) {
                throw std::invalid_argument("Invalid --color: missing value");
            }
            command.color_settings.mode = term_pcl::parseColorMode(argv[i]);
        } else if (arg == "--help" || arg == "-h") {
            printUsage(std::cout);
            std::exit(0);
        } else if (!arg.empty() && arg.front() == '-') {
            throw std::invalid_argument("Unknown option: " + arg);
        } else if (command.path.empty()) {
            command.path = arg;
        } else {
            throw std::invalid_argument("Unexpected argument: " + arg);
        }
    }

    if (command.path.empty()) {
        throw std::invalid_argument("Missing point cloud file");
    }
    if (command.index_mode && command.output_path.empty()) {
        throw std::invalid_argument("Missing --output value");
    }
    if (!command.index_mode && command.force) {
        throw std::invalid_argument("--force is only valid with the index subcommand");
    }
    return command;
}

void printTermcloudCheckSummary(std::ostream& stream, const term_pcl::TermcloudHierarchyMetadata& metadata) {
    stream << "Termcloud v3 index: " << metadata.index.frame_id << "\n"
           << "points=" << metadata.index.point_count << "\n"
           << "nodes=" << metadata.index.nodes.size() << "\n"
           << "has_rgb=" << (metadata.has_rgb ? 1 : 0) << "\n"
           << "bounds=x[" << metadata.index.min_x << ',' << metadata.index.max_x << "] "
           << "y[" << metadata.index.min_y << ',' << metadata.index.max_y << "] "
           << "z[" << metadata.index.min_z << ',' << metadata.index.max_z << "]\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const auto command = parseCommandLine(argc, argv);

        if (command.check_only && term_pcl::isTermcloudV3(command.path)) {
            const auto start = std::chrono::steady_clock::now();
            term_pcl::validateTermcloudHierarchy(command.path);
            const auto metadata = term_pcl::readTermcloudHierarchyMetadata(command.path);
            const auto end = std::chrono::steady_clock::now();
            if (command.load_options.profile) {
                const auto load_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                std::cerr << "load_ms=" << load_ms << " points=" << metadata.index.point_count << "\n";
            }
            printTermcloudCheckSummary(std::cout, metadata);
            return 0;
        }

        const auto start = std::chrono::steady_clock::now();
        auto cloud = term_pcl::loadPointCloud(command.path, command.load_options);
        const auto end = std::chrono::steady_clock::now();

        if (command.load_options.profile) {
            const auto load_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cerr << "load_ms=" << load_ms << " points=" << cloud->points.size() << "\n";
        }

        if (command.index_mode) {
            term_pcl::TermcloudWriteOptions options;
            options.force = command.force;
            term_pcl::writeTermcloudV3Index(*cloud, command.output_path, options);
            std::cout << "Indexed " << cloud->points.size() << " points to " << command.output_path << "\n";
            return 0;
        }

        if (command.check_only) {
            std::cout << "Loaded " << cloud->points.size() << " points from " << cloud->frame_id << "\n";
            return 0;
        }

        std::shared_ptr<term_pcl::CloudProvider> provider;
        if (term_pcl::isTermcloudV3(command.path)) {
            auto metadata = term_pcl::readTermcloudHierarchyMetadata(command.path);
            const auto cache_budget = command.load_options.point_budget > 0 ? command.load_options.point_budget * 4 : 200000;
            provider = std::make_shared<term_pcl::HierarchicalLodProvider>(
                std::move(metadata.index), command.path, cache_budget, std::thread::hardware_concurrency());
        } else if (command.load_options.point_budget > 0) {
            provider = std::make_shared<term_pcl::LodCloudProvider>(cloud, command.load_options.point_budget);
        } else {
            provider = std::make_shared<term_pcl::StaticCloudProvider>(cloud);
        }
        term_pcl::Visualizer visualizer(provider, command.color_settings);
        visualizer.run();
        return 0;
    } catch (const term_pcl::LoadError& error) {
        std::cerr << "term-pcl: " << error.what() << "\n";
        return 1;
    } catch (const std::invalid_argument& error) {
        std::cerr << "term-pcl: " << error.what() << "\n";
        printUsage(std::cerr);
        return 2;
    } catch (const std::filesystem::filesystem_error& error) {
        std::cerr << "term-pcl: filesystem error: " << error.what() << "\n";
        return 1;
    } catch (const std::exception& error) {
        std::cerr << "term-pcl: unexpected error: " << error.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "term-pcl: unexpected non-standard error\n";
        return 1;
    }
}
