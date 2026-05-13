#include "term_pcl/lod_cloud_provider.hpp"
#include "term_pcl/cloud_optimizer.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using term_pcl::CloudData;
using term_pcl::Point;
using term_pcl::RenderRequest;
using term_pcl::StaticCloudProvider;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_close(float actual, float expected, const std::string& message) {
    require(std::fabs(actual - expected) < 0.0001f, message);
}

std::shared_ptr<CloudData> makeCloud() {
    auto cloud = std::make_shared<CloudData>();
    cloud->frame_id = "lod-source";
    for (int i = 0; i < 1000; ++i) {
        cloud->points.push_back(Point{static_cast<float>(i), static_cast<float>(i % 10), static_cast<float>(i % 7)});
    }
    term_pcl::recomputeStats(*cloud);
    return cloud;
}

void test_static_provider_accepts_render_request() {
    auto cloud = std::make_shared<CloudData>();
    cloud->frame_id = "static";
    cloud->points = {Point{0.0f, 0.0f, 0.0f}};

    StaticCloudProvider provider(cloud);
    RenderRequest request;
    request.point_budget = 1;
    request.screen_width = 80;
    request.screen_height = 24;

    auto selected = provider.get_data(request);
    require(selected->frame_id == "static", "static provider should preserve frame id for render request");
    require(selected->points.size() == 1, "static provider should return data for render request");
}

void test_lod_provider_uses_request_budget() {
    auto cloud = std::make_shared<CloudData>();
    cloud->frame_id = "request";
    for (int i = 0; i < 50; ++i) {
        cloud->points.push_back(Point{static_cast<float>(i), 0.0f, 0.0f});
    }
    term_pcl::recomputeStats(*cloud);

    term_pcl::LodCloudProvider provider(cloud, 30);
    RenderRequest request;
    request.point_budget = 10;
    request.screen_width = 80;
    request.screen_height = 24;

    auto selected = provider.get_data(request);
    require(selected->points.size() == 10, "LOD provider should use request point budget");
    require(provider.stats().selected_points == 10, "LOD provider should report request-selected points");
}

}  // namespace

int main() {
    test_static_provider_accepts_render_request();
    test_lod_provider_uses_request_budget();
    const auto source = makeCloud();
    term_pcl::LodCloudProvider provider(source, 100);

    const auto first = provider.get_data();
    const auto second = provider.get_data();

    require(first->points.size() <= 100, "LOD provider should cap returned points to budget");
    require(first->points.size() == second->points.size(), "LOD provider should return stable point count");
    require_close(first->points.front().x, second->points.front().x, "LOD provider should return stable first point");
    require(first->frame_id == "lod-source", "LOD provider should preserve frame id");
    require_close(first->min_x, source->min_x, "LOD provider should preserve source min x");
    require_close(first->max_x, source->max_x, "LOD provider should preserve source max x");
    require_close(first->cx, source->cx, "LOD provider should preserve source center x");

    term_pcl::LodCloudProvider no_budget(source, 0);
    require(no_budget.get_data()->points.size() == source->points.size(), "zero budget should return original cloud");

    std::cout << "lod cloud provider tests passed\n";
    return 0;
}
