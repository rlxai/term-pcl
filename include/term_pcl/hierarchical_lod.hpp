#ifndef TERM_PCL_HIERARCHICAL_LOD_HPP_
#define TERM_PCL_HIERARCHICAL_LOD_HPP_

#include "term_pcl/chunk_cache.hpp"
#include "term_pcl/chunk_loader.hpp"
#include "term_pcl/cloud_provider.hpp"
#include "term_pcl/types.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace term_pcl {

struct LodNode {
    std::size_t id = 0;
    std::size_t level = 0;
    std::size_t first_point = 0;
    std::size_t point_count = 0;
    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
    float min_z = 0.0f;
    float max_z = 0.0f;
    std::vector<std::size_t> children;
    std::string chunk_file;
};

struct HierarchicalLodIndex {
    std::string frame_id;
    std::size_t root_node = 0;
    std::size_t point_count = 0;
    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
    float min_z = 0.0f;
    float max_z = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float cz = 0.0f;
    std::vector<LodNode> nodes;
};

struct LodSelectionSettings {
    float near_plane = -1000.0f;
    float far_plane = 1000.0f;
    float screen_error_threshold = 25.0f;
};

HierarchicalLodIndex buildHierarchicalLodIndex(
    const term_pcl::CloudData& cloud,
    std::size_t leaf_point_limit,
    std::size_t max_depth);

std::vector<std::size_t> selectHierarchicalLodNodes(
    const HierarchicalLodIndex& index,
    const term_pcl::RenderRequest& request,
    const LodSelectionSettings& settings = {});

class HierarchicalLodProvider : public term_pcl::CloudProvider {
public:
    HierarchicalLodProvider(HierarchicalLodIndex index,
                            std::string base_path,
                            std::size_t cache_point_budget,
                            std::size_t worker_count);

    std::shared_ptr<const term_pcl::CloudData> get_data() const override;
    std::shared_ptr<const term_pcl::CloudData> get_data(
        const term_pcl::RenderRequest& request) const override;
    term_pcl::RenderStats stats() const override;

private:
    void drain_completed() const;
    std::string chunk_path(const LodNode& node) const;

    HierarchicalLodIndex index_;
    std::string base_path_;
    mutable ChunkCache cache_;
    mutable ChunkLoader loader_;
    mutable std::shared_ptr<term_pcl::CloudData> last_cloud_;
    mutable term_pcl::RenderStats last_stats_;
    mutable std::size_t failed_chunks_ = 0;
    mutable std::string last_chunk_error_;
};

}  // namespace term_pcl

#endif  // TERM_PCL_HIERARCHICAL_LOD_HPP_
