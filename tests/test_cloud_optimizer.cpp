#include "term_pcl/cloud_optimizer.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

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

std::shared_ptr<CloudData> makeCloud(int count) {
    auto cloud = std::make_shared<CloudData>();
    cloud->frame_id = "synthetic";
    for (int i = 0; i < count; ++i) {
        cloud->points.push_back(Point{static_cast<float>(i), static_cast<float>(i % 3), static_cast<float>(-i)});
    }
    term_pcl::recomputeStats(*cloud);
    return cloud;
}

void test_point_budget_preserves_rgb_fields() {
    CloudData cloud;
    for (int i = 0; i < 4; ++i) {
        Point point{static_cast<float>(i), 0.0f, 0.0f};
        point.has_color = true;
        point.r = static_cast<unsigned char>(10 + i);
        point.g = static_cast<unsigned char>(20 + i);
        point.b = static_cast<unsigned char>(30 + i);
        cloud.points.push_back(point);
    }
    term_pcl::recomputeStats(cloud);
    term_pcl::applyPointBudget(cloud, 2);
    require(cloud.points.size() == 2, "Budget should reduce point count");
    require(cloud.points[0].has_color && cloud.points[1].has_color, "Budget should preserve color flag");
}

}  // namespace

int main() {
    test_point_budget_preserves_rgb_fields();

    auto cloud = makeCloud(10);
    term_pcl::applyPointBudget(*cloud, 4);

    require(cloud->points.size() == 4, "point budget should keep requested point count");
    require_close(cloud->points[0].x, 0.0f, "budget should keep first point");
    require_close(cloud->points[1].x, 3.0f, "budget should sample deterministic middle point 1");
    require_close(cloud->points[2].x, 6.0f, "budget should sample deterministic middle point 2");
    require_close(cloud->points[3].x, 9.0f, "budget should keep last point");
    require_close(cloud->min_x, 0.0f, "budget should recompute min x");
    require_close(cloud->max_x, 9.0f, "budget should recompute max x");
    require_close(cloud->cx, 4.5f, "budget should recompute center x");
    require_close(cloud->min_z, -9.0f, "budget should recompute min z");
    require_close(cloud->max_z, 0.0f, "budget should recompute max z");

    auto unchanged = makeCloud(3);
    term_pcl::applyPointBudget(*unchanged, 10);
    require(unchanged->points.size() == 3, "budget larger than cloud should keep all points");

    std::cout << "cloud optimizer tests passed\n";
    return 0;
}
