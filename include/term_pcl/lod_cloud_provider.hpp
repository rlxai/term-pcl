#ifndef TERM_PCL_LOD_CLOUD_PROVIDER_HPP_
#define TERM_PCL_LOD_CLOUD_PROVIDER_HPP_

#include <cstddef>
#include <memory>

#include "term_pcl/spatial_lod.hpp"
#include "term_pcl/cloud_provider.hpp"

namespace term_pcl {

class LodCloudProvider : public term_pcl::CloudProvider {
public:
    LodCloudProvider(std::shared_ptr<const term_pcl::CloudData> source, std::size_t point_budget);
    std::shared_ptr<const term_pcl::CloudData> get_data() const override;
    std::shared_ptr<const term_pcl::CloudData> get_data(
        const term_pcl::RenderRequest& request) const override;
    term_pcl::RenderStats stats() const override;

private:
    std::shared_ptr<const term_pcl::CloudData> source_;
    std::size_t point_budget_ = 0;
    SpatialLodProvider spatial_provider_;
};

}  // namespace term_pcl

#endif  // TERM_PCL_LOD_CLOUD_PROVIDER_HPP_
