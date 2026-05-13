#include "term_pcl/hierarchical_lod.hpp"
#include "term_pcl/termcloud_index.hpp"

#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using term_pcl::HierarchicalLodProvider;
using term_pcl::LodSelectionSettings;
using term_pcl::buildHierarchicalLodIndex;
using term_pcl::selectHierarchicalLodNodes;
using term_pcl::CloudData;
using term_pcl::Point;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

CloudData make_grid_cloud() {
    CloudData cloud;
    cloud.frame_id = "grid";
    for (int x = 0; x < 4; ++x) {
        for (int y = 0; y < 4; ++y) {
            cloud.points.push_back(Point{static_cast<float>(x), static_cast<float>(y), 0.0f});
        }
    }
    cloud.min_x = 0.0f;
    cloud.max_x = 3.0f;
    cloud.min_y = 0.0f;
    cloud.max_y = 3.0f;
    cloud.min_z = 0.0f;
    cloud.max_z = 0.0f;
    cloud.cx = 1.5f;
    cloud.cy = 1.5f;
    cloud.cz = 0.0f;
    return cloud;
}

void test_hierarchy_builder_creates_root_and_children() {
    const auto cloud = make_grid_cloud();
    const auto index = buildHierarchicalLodIndex(cloud, 4, 2);

    require(index.nodes.size() > 1, "Hierarchy should contain root and child nodes");
    require(index.root_node == 0, "Root node should be index 0");
    require(index.nodes[0].level == 0, "Root node should be level 0");
    require(index.nodes[0].point_count == cloud.points.size(), "Root should cover all source points");
    require(!index.nodes[0].children.empty(), "Root should have children for a multi-node cloud");
    require(index.nodes[0].min_x == 0.0f && index.nodes[0].max_x == 3.0f, "Root x bounds should match cloud");
    require(index.nodes[0].min_y == 0.0f && index.nodes[0].max_y == 3.0f, "Root y bounds should match cloud");
}

void test_leaf_nodes_respect_point_limit() {
    const auto cloud = make_grid_cloud();
    const auto index = buildHierarchicalLodIndex(cloud, 4, 2);

    for (const auto& node : index.nodes) {
        if (node.children.empty()) {
            require(node.point_count <= 4, "Leaf node should not exceed requested leaf point count");
        }
    }
}

void test_selection_skips_nodes_behind_camera() {
    auto cloud = make_grid_cloud();
    cloud.points.push_back(Point{-10.0f, 0.0f, 0.0f});
    const auto index = buildHierarchicalLodIndex(cloud, 1, 3);

    term_pcl::RenderRequest request;
    request.forward = 0.0f;
    request.point_budget = 16;
    request.zoom = 100.0f;

    LodSelectionSettings settings;
    settings.near_plane = -1.0f;
    settings.far_plane = 20.0f;
    const auto selected = selectHierarchicalLodNodes(index, request, settings);

    require(!selected.empty(), "Selection should keep visible nodes");
    for (const auto node_id : selected) {
        require(index.nodes[node_id].max_x >= settings.near_plane, "Selection should skip nodes fully behind near plane");
    }
}

void test_close_zoomed_view_selects_finer_nodes_than_far_view() {
    const auto cloud = make_grid_cloud();
    const auto index = buildHierarchicalLodIndex(cloud, 2, 4);

    LodSelectionSettings settings;
    settings.screen_error_threshold = 10.0f;

    term_pcl::RenderRequest far_request;
    far_request.forward = -20.0f;
    far_request.zoom = 20.0f;
    far_request.point_budget = 16;

    term_pcl::RenderRequest near_request;
    near_request.forward = -1.0f;
    near_request.zoom = 200.0f;
    near_request.point_budget = 16;

    const auto far_nodes = selectHierarchicalLodNodes(index, far_request, settings);
    const auto near_nodes = selectHierarchicalLodNodes(index, near_request, settings);

    require(!far_nodes.empty(), "Far view should select at least one node");
    require(near_nodes.size() > far_nodes.size(), "Near zoomed view should select finer/more nodes than far view");
}

void test_provider_returns_available_coarse_data_and_reports_streaming_stats() {
    const auto dir = std::filesystem::temp_directory_path() / "hierarchical_provider_test.termcloud";
    std::filesystem::remove_all(dir);
    const auto cloud = make_grid_cloud();
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), 2, 3);
    auto metadata = term_pcl::readTermcloudHierarchyMetadata(dir.string());

    HierarchicalLodProvider provider(std::move(metadata.index), dir.string(), 8, 1);

    term_pcl::RenderRequest request;
    request.point_budget = 8;
    request.zoom = 200.0f;
    request.forward = -1.0f;

    const auto first = provider.get_data(request);
    const auto first_stats = provider.stats();
    require(first != nullptr, "Provider should always return a cloud object");
    require(first_stats.visible_chunks > 0, "Provider should report visible chunks");
    require(first_stats.queued_chunks > 0 || first_stats.cached_chunks > 0, "Provider should queue or cache visible chunks");

    for (int i = 0; i < 50 && provider.stats().cached_chunks == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        provider.get_data(request);
    }

    const auto refined = provider.get_data(request);
    const auto refined_stats = provider.stats();
    require(!refined->points.empty(), "Provider should return loaded points after async refinement");
    require(refined_stats.cached_chunks > 0, "Provider should report cached chunks after loading");

    std::filesystem::remove_all(dir);
}

void test_provider_preserves_rgb_fields_in_loaded_chunks() {
    const auto dir = std::filesystem::temp_directory_path() / "hierarchical_provider_rgb_test.termcloud";
    std::filesystem::remove_all(dir);
    auto cloud = make_grid_cloud();
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        auto& point = cloud.points[i];
        point.has_color = true;
        point.r = static_cast<unsigned char>(10 + i);
        point.g = static_cast<unsigned char>(20 + i);
        point.b = static_cast<unsigned char>(30 + i);
    }
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), 2, 3);
    auto metadata = term_pcl::readTermcloudHierarchyMetadata(dir.string());

    HierarchicalLodProvider provider(std::move(metadata.index), dir.string(), 8, 1);

    term_pcl::RenderRequest request;
    request.point_budget = 8;
    request.zoom = 200.0f;
    request.forward = -1.0f;

    for (int i = 0; i < 50 && provider.stats().cached_chunks == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        provider.get_data(request);
    }

    const auto selected = provider.get_data(request);
    require(!selected->points.empty(), "Provider RGB test should return loaded points");
    for (const auto& point : selected->points) {
        require(point.has_color, "Provider should preserve loaded point color flag");
    }

    std::filesystem::remove_all(dir);
}

void test_provider_reports_chunk_load_failures() {
    const auto dir = std::filesystem::temp_directory_path() / "hierarchical_provider_failure_test.termcloud";
    std::filesystem::remove_all(dir);
    const auto cloud = make_grid_cloud();
    term_pcl::writeTermcloudV3Index(cloud, dir.string(), 2, 3);
    auto metadata = term_pcl::readTermcloudHierarchyMetadata(dir.string());

    for (const auto& node : metadata.index.nodes) {
        if (node.children.empty()) {
            std::filesystem::remove(dir / node.chunk_file);
            break;
        }
    }

    HierarchicalLodProvider provider(std::move(metadata.index), dir.string(), 8, 1);

    term_pcl::RenderRequest request;
    request.point_budget = 8;
    request.zoom = 200.0f;
    request.forward = -1.0f;

    for (int i = 0; i < 50 && provider.stats().failed_chunks == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        provider.get_data(request);
    }

    const auto stats = provider.stats();
    require(stats.failed_chunks > 0, "Provider should report failed chunk loads");
    require(stats.last_chunk_error.find("Failed to read chunk") != std::string::npos,
            "Provider should expose the last chunk load error");

    std::filesystem::remove_all(dir);
}

int main() {
    test_hierarchy_builder_creates_root_and_children();
    test_leaf_nodes_respect_point_limit();
    test_selection_skips_nodes_behind_camera();
    test_close_zoomed_view_selects_finer_nodes_than_far_view();
    test_provider_returns_available_coarse_data_and_reports_streaming_stats();
    test_provider_preserves_rgb_fields_in_loaded_chunks();
    test_provider_reports_chunk_load_failures();
    return 0;
}
