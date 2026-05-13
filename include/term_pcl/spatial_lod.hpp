#pragma once

#include "term_pcl/cloud_provider.hpp"
#include "term_pcl/types.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace term_pcl {

struct SpatialChunk {
    std::size_t first = 0;
    std::size_t count = 0;
    std::uint64_t morton_min = 0;
    std::uint64_t morton_max = 0;
    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
    float min_z = 0.0f;
    float max_z = 0.0f;
};

std::uint64_t mortonCodeForPoint(const term_pcl::Point& point,
                                 const term_pcl::CloudData& bounds);

class SpatialLodProvider : public term_pcl::CloudProvider {
public:
    SpatialLodProvider(std::shared_ptr<const term_pcl::CloudData> source,
                       std::size_t point_budget);

    std::shared_ptr<const term_pcl::CloudData> get_data() const override;
    std::shared_ptr<const term_pcl::CloudData> get_data(
        const term_pcl::RenderRequest& request) const override;
    term_pcl::RenderStats stats() const override;

    const std::vector<SpatialChunk>& chunks() const;

private:
    std::shared_ptr<const term_pcl::CloudData> source_;
    std::shared_ptr<term_pcl::CloudData> ordered_;
    std::vector<SpatialChunk> chunks_;
    std::size_t point_budget_ = 0;
    mutable std::shared_ptr<term_pcl::CloudData> last_selection_;
    mutable term_pcl::RenderStats last_stats_;
};

}  // namespace term_pcl
