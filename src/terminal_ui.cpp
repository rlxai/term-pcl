#include "term_pcl/terminal_ui.hpp"

#include <array>
#include <iomanip>
#include <sstream>

namespace term_pcl {
namespace {

std::string compactCount(std::size_t count) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << static_cast<double>(count) / 1000.0 << 'k';
    return stream.str();
}

}  // namespace

std::string progressPulse(std::size_t frame_index) {
    static constexpr std::array<const char*, 4> kFrames = {"◜", "◝", "◞", "◟"};
    return kFrames[frame_index % kFrames.size()];
}

std::string formatMotionState(const AnimationState& state) {
    if (state.moving) {
        return "refining " + progressPulse(state.frame_index);
    }
    return "stable ◆";
}

std::string terminalRestoreSequence() {
    return "\x1b[?1000l\x1b[?1002l\x1b[?1003l\x1b[?1006l";
}

std::string formatHudLine(const RenderStats& stats, float frame_ms) {
    const auto fps = frame_ms > 0.0f ? static_cast<int>(1000.0f / frame_ms) : 0;
    std::ostringstream stream;
    stream << "● LOD " << compactCount(stats.selected_points) << '/' << compactCount(stats.source_points)
           << " pts  ◇ chunks " << stats.chunks_selected << '/' << stats.chunks_total;
    if (stats.visible_chunks > 0 || stats.cached_chunks > 0 || stats.queued_chunks > 0) {
        stream << " visible " << stats.visible_chunks
               << " cache " << stats.cached_chunks
               << " queued " << stats.queued_chunks;
    }
    stream << "  ◷ " << fps << " fps";
    if (stats.progressive) {
        stream << "  progressive";
    }
    return stream.str();
}

}  // namespace term_pcl
