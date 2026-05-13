#ifndef TERMINAL_PCL_VISUALIZER_TERMINAL_UI_HPP_
#define TERMINAL_PCL_VISUALIZER_TERMINAL_UI_HPP_

#include "term_pcl/cloud_provider.hpp"

#include <cstddef>
#include <string>

namespace term_pcl {

struct AnimationState {
    bool moving = false;
    std::size_t frame_index = 0;
};

std::string formatHudLine(const RenderStats& stats, float frame_ms);
std::string formatMotionState(const AnimationState& state);
std::string progressPulse(std::size_t frame_index);
std::string terminalRestoreSequence();

}  // namespace term_pcl

#endif  // TERMINAL_PCL_VISUALIZER_TERMINAL_UI_HPP_
