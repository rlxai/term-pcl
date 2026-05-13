#include "term_pcl/cloud_provider.hpp"

#include <utility>

namespace term_pcl {

StaticCloudProvider::StaticCloudProvider(std::shared_ptr<const CloudData> data) : data_(std::move(data)) {}

std::shared_ptr<const CloudData> CloudProvider::get_data(const RenderRequest&) const {
    return get_data();
}

RenderStats CloudProvider::stats() const {
    return {};
}

std::shared_ptr<const CloudData> StaticCloudProvider::get_data() const {
    return data_;
}

std::shared_ptr<const CloudData> StaticCloudProvider::get_data(const RenderRequest&) const {
    return data_;
}

RenderStats StaticCloudProvider::stats() const {
    RenderStats result;
    if (data_) {
        result.source_points = data_->points.size();
        result.selected_points = data_->points.size();
    }
    return result;
}

}  // namespace term_pcl
