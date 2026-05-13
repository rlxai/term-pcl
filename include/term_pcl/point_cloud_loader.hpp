#ifndef TERM_PCL_POINT_CLOUD_LOADER_HPP_
#define TERM_PCL_POINT_CLOUD_LOADER_HPP_

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

#include "term_pcl/types.hpp"

namespace term_pcl {

class LoadError : public std::runtime_error {
public:
    explicit LoadError(const std::string& message);
};

struct LoadOptions {
    std::size_t point_budget = 0;
    float voxel_size = 0.0f;
    bool profile = false;
};

bool isTermcloudV3(const std::string& path);
std::shared_ptr<term_pcl::CloudData> loadPointCloud(const std::string& path, const LoadOptions& options = {});

}  // namespace term_pcl

#endif  // TERM_PCL_POINT_CLOUD_LOADER_HPP_
