#include "term_pcl/spatial_lod.hpp"

#include "term_pcl/cloud_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace term_pcl {
namespace {

constexpr std::size_t kChunkSize = 4096;
constexpr float kMovingBudgetFraction = 0.4f;

std::uint32_t quantize(float value, float min_value, float max_value) {
    if (max_value <= min_value) {
        return 0;
    }
    const float normalized = (value - min_value) / (max_value - min_value);
    const float clamped = std::max(0.0f, std::min(1.0f, normalized));
    return static_cast<std::uint32_t>(std::round(clamped * 2097151.0f));
}

std::uint64_t expandBits(std::uint32_t value) {
    std::uint64_t x = value & 0x1fffff;
    x = (x | x << 32) & 0x1f00000000ffffULL;
    x = (x | x << 16) & 0x1f0000ff0000ffULL;
    x = (x | x << 8) & 0x100f00f00f00f00fULL;
    x = (x | x << 4) & 0x10c30c30c30c30c3ULL;
    x = (x | x << 2) & 0x1249249249249249ULL;
    return x;
}

SpatialChunk makeChunk(const term_pcl::CloudData& cloud, std::size_t first, std::size_t count) {
    SpatialChunk chunk;
    chunk.first = first;
    chunk.count = count;
    chunk.min_x = chunk.min_y = chunk.min_z = std::numeric_limits<float>::max();
    chunk.max_x = chunk.max_y = chunk.max_z = std::numeric_limits<float>::lowest();
    chunk.morton_min = std::numeric_limits<std::uint64_t>::max();
    chunk.morton_max = 0;

    for (std::size_t i = first; i < first + count; ++i) {
        const auto& point = cloud.points[i];
        const auto morton = mortonCodeForPoint(point, cloud);
        chunk.morton_min = std::min(chunk.morton_min, morton);
        chunk.morton_max = std::max(chunk.morton_max, morton);
        chunk.min_x = std::min(chunk.min_x, point.x);
        chunk.max_x = std::max(chunk.max_x, point.x);
        chunk.min_y = std::min(chunk.min_y, point.y);
        chunk.max_y = std::max(chunk.max_y, point.y);
        chunk.min_z = std::min(chunk.min_z, point.z);
        chunk.max_z = std::max(chunk.max_z, point.z);
    }
    return chunk;
}

std::size_t effectiveBudget(std::size_t provider_budget, const term_pcl::RenderRequest& request) {
    std::size_t budget = request.point_budget > 0 ? request.point_budget : provider_budget;
    if (provider_budget > 0) {
        budget = budget > 0 ? std::min(budget, provider_budget) : provider_budget;
    }
    if (request.screen_width > 0 && request.screen_height > 0) {
        const auto screen_budget = static_cast<std::size_t>(request.screen_width) *
                                   static_cast<std::size_t>(request.screen_height) * 8;
        budget = budget > 0 ? std::min(budget, screen_budget) : screen_budget;
    }
    if (request.moving && budget > 0) {
        budget = std::max<std::size_t>(1, static_cast<std::size_t>(std::round(static_cast<float>(budget) * kMovingBudgetFraction)));
    }
    return budget;
}

}  // namespace

std::uint64_t mortonCodeForPoint(const term_pcl::Point& point,
                                 const term_pcl::CloudData& bounds) {
    const auto x = quantize(point.x, bounds.min_x, bounds.max_x);
    const auto y = quantize(point.y, bounds.min_y, bounds.max_y);
    const auto z = quantize(point.z, bounds.min_z, bounds.max_z);
    return expandBits(x) | (expandBits(y) << 1) | (expandBits(z) << 2);
}

SpatialLodProvider::SpatialLodProvider(std::shared_ptr<const term_pcl::CloudData> source,
                                       std::size_t point_budget)
    : source_(std::move(source)), ordered_(std::make_shared<term_pcl::CloudData>()), point_budget_(point_budget) {
    if (!source_) {
        return;
    }

    *ordered_ = *source_;
    std::sort(ordered_->points.begin(), ordered_->points.end(), [this](const auto& lhs, const auto& rhs) {
        return mortonCodeForPoint(lhs, *source_) < mortonCodeForPoint(rhs, *source_);
    });
    recomputeStats(*ordered_);

    for (std::size_t first = 0; first < ordered_->points.size(); first += kChunkSize) {
        const auto count = std::min(kChunkSize, ordered_->points.size() - first);
        chunks_.push_back(makeChunk(*ordered_, first, count));
    }
}

std::shared_ptr<const term_pcl::CloudData> SpatialLodProvider::get_data() const {
    term_pcl::RenderRequest request;
    request.point_budget = point_budget_;
    return get_data(request);
}

std::shared_ptr<const term_pcl::CloudData> SpatialLodProvider::get_data(
    const term_pcl::RenderRequest& request) const {
    if (!ordered_) {
        return source_;
    }

    const auto budget = effectiveBudget(point_budget_, request);
    if (budget == 0 || budget >= ordered_->points.size()) {
        last_selection_ = ordered_;
        last_stats_.source_points = source_ ? source_->points.size() : 0;
        last_stats_.selected_points = ordered_->points.size();
        last_stats_.chunks_total = chunks_.size();
        last_stats_.chunks_selected = chunks_.size();
        last_stats_.progressive = request.moving;
        return ordered_;
    }

    auto selected = std::make_shared<term_pcl::CloudData>();
    selected->frame_id = ordered_->frame_id;
    selected->min_x = ordered_->min_x;
    selected->max_x = ordered_->max_x;
    selected->min_y = ordered_->min_y;
    selected->max_y = ordered_->max_y;
    selected->min_z = ordered_->min_z;
    selected->max_z = ordered_->max_z;
    selected->cx = ordered_->cx;
    selected->cy = ordered_->cy;
    selected->cz = ordered_->cz;
    selected->points.reserve(budget);

    std::vector<bool> selected_chunks(chunks_.size(), false);
    std::size_t selected_chunk_count = 0;
    const double stride = budget > 1 ? static_cast<double>(ordered_->points.size() - 1) / static_cast<double>(budget - 1) : 0.0;
    for (std::size_t i = 0; i < budget; ++i) {
        const auto index = std::min<std::size_t>(ordered_->points.size() - 1,
                                                static_cast<std::size_t>(std::round(static_cast<double>(i) * stride)));
        selected->points.push_back(ordered_->points[index]);
        const auto chunk_index = index / kChunkSize;
        if (chunk_index < selected_chunks.size() && !selected_chunks[chunk_index]) {
            selected_chunks[chunk_index] = true;
            ++selected_chunk_count;
        }
    }

    last_selection_ = selected;
    last_stats_.source_points = source_ ? source_->points.size() : 0;
    last_stats_.selected_points = selected->points.size();
    last_stats_.chunks_total = chunks_.size();
    last_stats_.chunks_selected = selected_chunk_count;
    last_stats_.progressive = request.moving;
    return last_selection_;
}

term_pcl::RenderStats SpatialLodProvider::stats() const {
    return last_stats_;
}

const std::vector<SpatialChunk>& SpatialLodProvider::chunks() const {
    return chunks_;
}

}  // namespace term_pcl
