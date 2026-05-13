#ifndef TERM_PCL_TERMCLOUD_INDEX_HPP_
#define TERM_PCL_TERMCLOUD_INDEX_HPP_

#include <memory>
#include <string>

#include "term_pcl/hierarchical_lod.hpp"
#include "term_pcl/types.hpp"

namespace term_pcl {

struct TermcloudHierarchyMetadata {
    HierarchicalLodIndex index;
    std::string base_path;
    bool has_rgb = false;
};

struct TermcloudWriteOptions {
    bool force = false;
};

void writeTermcloudIndex(const term_pcl::CloudData& cloud,
                         const std::string& output_path,
                         const TermcloudWriteOptions& options = {});
void writeTermcloudV3Index(const term_pcl::CloudData& cloud,
                           const std::string& output_path,
                           std::size_t leaf_point_limit = 4096,
                           std::size_t max_depth = 8);
void writeTermcloudV3Index(const term_pcl::CloudData& cloud,
                           const std::string& output_path,
                           const TermcloudWriteOptions& options,
                           std::size_t leaf_point_limit = 4096,
                           std::size_t max_depth = 8);
std::shared_ptr<term_pcl::CloudData> readTermcloudIndex(const std::string& input_path);
TermcloudHierarchyMetadata readTermcloudHierarchyMetadata(const std::string& path);
void validateTermcloudHierarchy(const std::string& path);

}  // namespace term_pcl

#endif  // TERM_PCL_TERMCLOUD_INDEX_HPP_
