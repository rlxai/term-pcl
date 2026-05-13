#include "term_pcl/lod_cloud_provider.hpp"

#include <utility>

namespace term_pcl {

LodCloudProvider::LodCloudProvider(std::shared_ptr<const term_pcl::CloudData> source,
                                   std::size_t point_budget)
    : source_(std::move(source)), point_budget_(point_budget), spatial_provider_(source_, point_budget_) {}

std::shared_ptr<const term_pcl::CloudData> LodCloudProvider::get_data() const {
    return spatial_provider_.get_data();
}

std::shared_ptr<const term_pcl::CloudData> LodCloudProvider::get_data(
    const term_pcl::RenderRequest& request) const {
    return spatial_provider_.get_data(request);
}

term_pcl::RenderStats LodCloudProvider::stats() const {
    return spatial_provider_.stats();
}

}  // namespace term_pcl
