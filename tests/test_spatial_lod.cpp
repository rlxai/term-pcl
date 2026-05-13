#include "term_pcl/spatial_lod.hpp"
#include "term_pcl/cloud_optimizer.hpp"

#include <memory>
#include <stdexcept>
#include <string>

using term_pcl::CloudData;
using term_pcl::Point;
using term_pcl::RenderRequest;
using term_pcl::SpatialLodProvider;
using term_pcl::mortonCodeForPoint;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_morton_code_preserves_spatial_order() {
    CloudData cloud;
    cloud.points = {Point{0, 0, 0}, Point{10, 0, 0}, Point{0, 10, 0}};
    term_pcl::recomputeStats(cloud);

    const auto a = mortonCodeForPoint(cloud.points[0], cloud);
    const auto b = mortonCodeForPoint(cloud.points[1], cloud);
    const auto c = mortonCodeForPoint(cloud.points[2], cloud);

    require(a != b, "Morton code should distinguish x-separated points");
    require(a != c, "Morton code should distinguish y-separated points");
}

void test_spatial_lod_respects_budget() {
    auto cloud = std::make_shared<CloudData>();
    cloud->frame_id = "grid";
    for (int i = 0; i < 100; ++i) {
        cloud->points.push_back(Point{static_cast<float>(i), 0.0f, 0.0f});
    }
    term_pcl::recomputeStats(*cloud);

    SpatialLodProvider provider(cloud, 20);
    RenderRequest request;
    request.point_budget = 12;
    request.screen_width = 80;
    request.screen_height = 24;

    auto selected = provider.get_data(request);
    require(selected->points.size() == 12, "Spatial LOD should respect request budget");
    require(selected->frame_id == "grid", "Spatial LOD should preserve frame id");

    auto stats = provider.stats();
    require(stats.source_points == 100, "Spatial LOD should report source points");
    require(stats.selected_points == 12, "Spatial LOD should report selected points");
    require(stats.chunks_total > 0, "Spatial LOD should report total chunks");
    require(stats.chunks_selected > 0, "Spatial LOD should report selected chunks");
}

void test_spatial_lod_preserves_source_bounds_for_budgeted_selection() {
    auto cloud = std::make_shared<CloudData>();
    cloud->frame_id = "bounds";
    for (int i = 0; i < 100; ++i) {
        cloud->points.push_back(Point{static_cast<float>(i), 0.0f, 0.0f});
    }
    term_pcl::recomputeStats(*cloud);

    SpatialLodProvider provider(cloud, 10);
    RenderRequest request;
    request.point_budget = 5;
    request.screen_width = 80;
    request.screen_height = 24;

    auto selected = provider.get_data(request);
    require(selected->min_x == cloud->min_x, "Spatial LOD should preserve source min x");
    require(selected->max_x == cloud->max_x, "Spatial LOD should preserve source max x");
    require(selected->cx == cloud->cx, "Spatial LOD should preserve source center x");
}

void test_spatial_lod_reports_selected_chunks() {
    auto cloud = std::make_shared<CloudData>();
    cloud->frame_id = "chunks";
    for (int i = 0; i < 9000; ++i) {
        cloud->points.push_back(Point{static_cast<float>(i), 0.0f, 0.0f});
    }
    term_pcl::recomputeStats(*cloud);

    SpatialLodProvider provider(cloud, 9000);
    RenderRequest request;
    request.point_budget = 2;
    request.screen_width = 80;
    request.screen_height = 24;

    provider.get_data(request);
    auto stats = provider.stats();
    require(stats.chunks_total == 3, "Spatial LOD should report all chunks");
    require(stats.chunks_selected == 2, "Spatial LOD should report only touched chunks");
}

void test_spatial_lod_reduces_budget_while_moving() {
    auto cloud = std::make_shared<CloudData>();
    cloud->frame_id = "moving";
    for (int i = 0; i < 100; ++i) {
        cloud->points.push_back(Point{static_cast<float>(i), 0.0f, 0.0f});
    }
    term_pcl::recomputeStats(*cloud);

    SpatialLodProvider provider(cloud, 40);
    RenderRequest request;
    request.point_budget = 40;
    request.screen_width = 80;
    request.screen_height = 24;
    request.moving = true;

    auto selected = provider.get_data(request);
    require(selected->points.size() == 16, "Spatial LOD should reduce moving budget");
    require(provider.stats().progressive, "Spatial LOD should report progressive moving state");
}

void test_spatial_lod_preserves_rgb_fields() {
    auto cloud = std::make_shared<CloudData>();
    cloud->frame_id = "rgb-grid";
    for (int i = 0; i < 10; ++i) {
        Point point{static_cast<float>(i), 0.0f, 0.0f};
        point.has_color = true;
        point.r = static_cast<unsigned char>(10 + i);
        point.g = static_cast<unsigned char>(20 + i);
        point.b = static_cast<unsigned char>(30 + i);
        cloud->points.push_back(point);
    }
    term_pcl::recomputeStats(*cloud);

    SpatialLodProvider provider(cloud, 10);
    RenderRequest request;
    request.point_budget = 4;
    request.screen_width = 80;
    request.screen_height = 24;

    auto selected = provider.get_data(request);
    require(selected->points.size() == 4, "Spatial LOD RGB test should select requested points");
    for (const auto& point : selected->points) {
        require(point.has_color, "Spatial LOD should preserve selected point color flag");
    }
}

int main() {
    test_morton_code_preserves_spatial_order();
    test_spatial_lod_respects_budget();
    test_spatial_lod_preserves_source_bounds_for_budgeted_selection();
    test_spatial_lod_reports_selected_chunks();
    test_spatial_lod_reduces_budget_while_moving();
    test_spatial_lod_preserves_rgb_fields();
    return 0;
}
