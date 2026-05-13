#include "term_pcl/cloud_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace term_pcl {

using term_pcl::CloudData;
using term_pcl::Point;

void recomputeStats(CloudData& cloud) {
    if (cloud.points.empty()) {
        cloud.cx = cloud.cy = cloud.cz = 0.0f;
        cloud.min_x = cloud.max_x = 0.0f;
        cloud.min_y = cloud.max_y = 0.0f;
        cloud.min_z = cloud.max_z = 0.0f;
        return;
    }

    cloud.min_x = cloud.max_x = cloud.points.front().x;
    cloud.min_y = cloud.max_y = cloud.points.front().y;
    cloud.min_z = cloud.max_z = cloud.points.front().z;

    for (const auto& point : cloud.points) {
        cloud.min_x = std::min(cloud.min_x, point.x);
        cloud.max_x = std::max(cloud.max_x, point.x);
        cloud.min_y = std::min(cloud.min_y, point.y);
        cloud.max_y = std::max(cloud.max_y, point.y);
        cloud.min_z = std::min(cloud.min_z, point.z);
        cloud.max_z = std::max(cloud.max_z, point.z);
    }

    cloud.cx = (cloud.min_x + cloud.max_x) * 0.5f;
    cloud.cy = (cloud.min_y + cloud.max_y) * 0.5f;
    cloud.cz = (cloud.min_z + cloud.max_z) * 0.5f;
}

void applyPointBudget(CloudData& cloud, std::size_t point_budget) {
    if (point_budget == 0 || cloud.points.size() <= point_budget) {
        return;
    }

    std::vector<Point> sampled;
    sampled.reserve(point_budget);
    const double stride = static_cast<double>(cloud.points.size() - 1) / static_cast<double>(point_budget - 1);
    std::size_t previous_index = cloud.points.size();

    for (std::size_t i = 0; i < point_budget; ++i) {
        auto index = static_cast<std::size_t>(std::round(static_cast<double>(i) * stride));
        index = std::min(index, cloud.points.size() - 1);
        if (index == previous_index && index + 1 < cloud.points.size()) {
            ++index;
        }
        sampled.push_back(cloud.points[index]);
        previous_index = index;
    }

    cloud.points = std::move(sampled);
    recomputeStats(cloud);
}

}  // namespace term_pcl
