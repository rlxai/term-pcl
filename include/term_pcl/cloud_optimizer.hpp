#ifndef TERM_PCL_CLOUD_OPTIMIZER_HPP_
#define TERM_PCL_CLOUD_OPTIMIZER_HPP_

#include <cstddef>

#include "term_pcl/types.hpp"

namespace term_pcl {

void recomputeStats(term_pcl::CloudData& cloud);
void applyPointBudget(term_pcl::CloudData& cloud, std::size_t point_budget);

}  // namespace term_pcl

#endif  // TERM_PCL_CLOUD_OPTIMIZER_HPP_
