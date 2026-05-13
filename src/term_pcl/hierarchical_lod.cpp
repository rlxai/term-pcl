#include "term_pcl/hierarchical_lod.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace term_pcl {
namespace {

using term_pcl::CloudData;

void set_node_bounds(LodNode& node, const CloudData& cloud, std::size_t first, std::size_t count) {
    if (count == 0) return;
    const auto& p0 = cloud.points[first];
    node.min_x = node.max_x = p0.x;
    node.min_y = node.max_y = p0.y;
    node.min_z = node.max_z = p0.z;
    for (std::size_t i = first + 1; i < first + count; ++i) {
        const auto& p = cloud.points[i];
        node.min_x = std::min(node.min_x, p.x);
        node.max_x = std::max(node.max_x, p.x);
        node.min_y = std::min(node.min_y, p.y);
        node.max_y = std::max(node.max_y, p.y);
        node.min_z = std::min(node.min_z, p.z);
        node.max_z = std::max(node.max_z, p.z);
    }
}

std::string chunk_file_for_id(std::size_t id) {
    return "chunks/" + std::to_string(id) + ".bin";
}

std::size_t add_node(HierarchicalLodIndex& index,
                     const CloudData& cloud,
                     std::size_t first,
                     std::size_t count,
                     std::size_t level,
                     std::size_t leaf_point_limit,
                     std::size_t max_depth) {
    LodNode node;
    node.id = index.nodes.size();
    node.level = level;
    node.first_point = first;
    node.point_count = count;
    node.chunk_file = chunk_file_for_id(node.id);
    set_node_bounds(node, cloud, first, count);
    index.nodes.push_back(std::move(node));
    const auto node_id = index.nodes.back().id;

    if (count > leaf_point_limit && level < max_depth) {
        const std::size_t child_count = 4;
        const std::size_t base = count / child_count;
        const std::size_t remainder = count % child_count;
        std::size_t child_first = first;
        for (std::size_t child = 0; child < child_count; ++child) {
            const std::size_t child_points = base + (child < remainder ? 1 : 0);
            if (child_points == 0) continue;
            const auto child_id = add_node(index, cloud, child_first, child_points, level + 1, leaf_point_limit, max_depth);
            index.nodes[node_id].children.push_back(child_id);
            child_first += child_points;
        }
    }

    return node_id;
}

float node_center_x(const LodNode& node) {
    return (node.min_x + node.max_x) * 0.5f;
}

float node_radius(const LodNode& node) {
    const float dx = node.max_x - node.min_x;
    const float dy = node.max_y - node.min_y;
    const float dz = node.max_z - node.min_z;
    return std::max(dx, std::max(dy, dz)) * 0.5f;
}

bool node_visible(const LodNode& node, const LodSelectionSettings& settings) {
    return node.max_x >= settings.near_plane && node.min_x <= settings.far_plane;
}

float projected_error(const LodNode& node, const term_pcl::RenderRequest& request) {
    const float camera_x = request.forward;
    const float distance = std::max(0.1f, std::abs(node_center_x(node) - camera_x));
    return node_radius(node) * std::max(1.0f, request.zoom) / distance;
}

void select_node(const HierarchicalLodIndex& index,
                 std::size_t node_id,
                 const term_pcl::RenderRequest& request,
                 const LodSelectionSettings& settings,
                 std::size_t& remaining_budget,
                 std::vector<std::size_t>& selected) {
    const auto& node = index.nodes[node_id];
    if (!node_visible(node, settings) || remaining_budget == 0) return;

    const bool should_refine = !request.moving && !node.children.empty() &&
                               projected_error(node, request) > settings.screen_error_threshold;
    if (should_refine) {
        const auto before = selected.size();
        for (const auto child_id : node.children) {
            select_node(index, child_id, request, settings, remaining_budget, selected);
            if (remaining_budget == 0) break;
        }
        if (selected.size() > before) return;
    }

    selected.push_back(node_id);
    remaining_budget = node.point_count >= remaining_budget ? 0 : remaining_budget - node.point_count;
}

}  // namespace

HierarchicalLodIndex buildHierarchicalLodIndex(const CloudData& cloud,
                                                std::size_t leaf_point_limit,
                                                std::size_t max_depth) {
    HierarchicalLodIndex index;
    index.frame_id = cloud.frame_id;
    index.point_count = cloud.points.size();
    index.min_x = cloud.min_x;
    index.max_x = cloud.max_x;
    index.min_y = cloud.min_y;
    index.max_y = cloud.max_y;
    index.min_z = cloud.min_z;
    index.max_z = cloud.max_z;
    index.cx = cloud.cx;
    index.cy = cloud.cy;
    index.cz = cloud.cz;
    if (!cloud.points.empty()) {
        index.root_node = add_node(index, cloud, 0, cloud.points.size(), 0,
                                  std::max<std::size_t>(1, leaf_point_limit), max_depth);
    }
    return index;
}

std::vector<std::size_t> selectHierarchicalLodNodes(const HierarchicalLodIndex& index,
                                                    const term_pcl::RenderRequest& request,
                                                    const LodSelectionSettings& settings) {
    std::vector<std::size_t> selected;
    if (index.nodes.empty()) return selected;
    std::size_t budget = request.point_budget > 0 ? request.point_budget : index.point_count;
    select_node(index, index.root_node, request, settings, budget, selected);
    return selected;
}

HierarchicalLodProvider::HierarchicalLodProvider(HierarchicalLodIndex index,
                                                 std::string base_path,
                                                 std::size_t cache_point_budget,
                                                 std::size_t worker_count)
    : index_(std::move(index)),
      base_path_(std::move(base_path)),
      cache_(cache_point_budget),
      loader_(worker_count),
      last_cloud_(std::make_shared<term_pcl::CloudData>()) {}

std::shared_ptr<const term_pcl::CloudData> HierarchicalLodProvider::get_data() const {
    term_pcl::RenderRequest request;
    request.point_budget = index_.point_count;
    return get_data(request);
}

std::shared_ptr<const term_pcl::CloudData> HierarchicalLodProvider::get_data(
    const term_pcl::RenderRequest& request) const {
    drain_completed();
    auto selected = selectHierarchicalLodNodes(index_, request);
    auto cloud = std::make_shared<term_pcl::CloudData>();
    cloud->frame_id = index_.frame_id;
    cloud->min_x = index_.min_x;
    cloud->max_x = index_.max_x;
    cloud->min_y = index_.min_y;
    cloud->max_y = index_.max_y;
    cloud->min_z = index_.min_z;
    cloud->max_z = index_.max_z;
    cloud->cx = index_.cx;
    cloud->cy = index_.cy;
    cloud->cz = index_.cz;

    term_pcl::RenderStats stats;
    stats.source_points = index_.point_count;
    stats.chunks_total = index_.nodes.size();
    stats.visible_chunks = selected.size();
    stats.progressive = request.moving;

    for (const auto node_id : selected) {
        const auto& node = index_.nodes[node_id];
        auto cached = cache_.get(node.id);
        if (cached.has_value()) {
            cloud->points.insert(cloud->points.end(), cached->begin(), cached->end());
            ++stats.chunks_selected;
        } else if (loader_.request(node.id, chunk_path(node))) {
            ++stats.queued_chunks;
        }
    }

    stats.cached_chunks = cache_.cached_chunks();
    stats.queued_chunks += loader_.queued_count();
    stats.failed_chunks = failed_chunks_;
    stats.last_chunk_error = last_chunk_error_;
    stats.selected_points = cloud->points.size();
    last_cloud_ = cloud;
    last_stats_ = stats;
    return last_cloud_;
}

term_pcl::RenderStats HierarchicalLodProvider::stats() const {
    return last_stats_;
}

void HierarchicalLodProvider::drain_completed() const {
    while (auto loaded = loader_.take_completed()) {
        if (loaded->ok) {
            cache_.put(loaded->chunk_id, std::move(loaded->points));
        } else {
            ++failed_chunks_;
            last_chunk_error_ = loaded->error;
        }
    }
}

std::string HierarchicalLodProvider::chunk_path(const LodNode& node) const {
    return base_path_ + "/" + node.chunk_file;
}

}  // namespace term_pcl
