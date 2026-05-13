#ifndef TERMINAL_PCL_VISUALIZER_CLOUD_PROVIDER_HPP_
#define TERMINAL_PCL_VISUALIZER_CLOUD_PROVIDER_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "term_pcl/types.hpp"

namespace term_pcl {

struct RenderRequest {
    int screen_width = 0;
    int screen_height = 0;
    std::size_t point_budget = 0;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float zoom = 1.0f;
    float pan_x = 0.0f;
    float pan_y = 0.0f;
    float forward = 0.0f;
    bool moving = false;
    std::size_t frame_index = 0;
};

struct RenderStats {
    std::size_t source_points = 0;
    std::size_t selected_points = 0;
    std::size_t chunks_total = 0;
    std::size_t chunks_selected = 0;
    std::size_t visible_chunks = 0;
    std::size_t cached_chunks = 0;
    std::size_t queued_chunks = 0;
    std::size_t failed_chunks = 0;
    std::string last_chunk_error;
    bool progressive = false;
};

class CloudProvider {
public:
    virtual ~CloudProvider() = default;
    virtual std::shared_ptr<const CloudData> get_data() const = 0;
    virtual std::shared_ptr<const CloudData> get_data(const RenderRequest& request) const;
    virtual RenderStats stats() const;
};

class StaticCloudProvider : public CloudProvider {
public:
    explicit StaticCloudProvider(std::shared_ptr<const CloudData> data);
    std::shared_ptr<const CloudData> get_data() const override;
    std::shared_ptr<const CloudData> get_data(const RenderRequest& request) const override;
    RenderStats stats() const override;

private:
    std::shared_ptr<const CloudData> data_;
};

}  // namespace term_pcl

#endif  // TERMINAL_PCL_VISUALIZER_CLOUD_PROVIDER_HPP_
