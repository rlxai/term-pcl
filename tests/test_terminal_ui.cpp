#include "term_pcl/terminal_ui.hpp"

#include <stdexcept>
#include <string>

using term_pcl::AnimationState;
using term_pcl::RenderStats;
using term_pcl::formatHudLine;
using term_pcl::formatMotionState;
using term_pcl::progressPulse;
using term_pcl::terminalRestoreSequence;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_hud_line_is_compact_and_informative() {
    RenderStats stats;
    stats.source_points = 100000;
    stats.selected_points = 24000;
    stats.chunks_total = 12;
    stats.chunks_selected = 4;
    stats.progressive = true;

    const auto hud = formatHudLine(stats, 16.7f);
    require(hud == "● LOD 24.0k/100.0k pts  ◇ chunks 4/12  ◷ 59 fps  progressive",
            "HUD line should format point, chunk, FPS, and progressive state");
}

void test_hud_line_includes_streaming_stats_when_present() {
    RenderStats stats;
    stats.source_points = 200000;
    stats.selected_points = 32000;
    stats.chunks_total = 40;
    stats.chunks_selected = 6;
    stats.visible_chunks = 8;
    stats.cached_chunks = 5;
    stats.queued_chunks = 3;
    stats.progressive = true;

    const auto hud = formatHudLine(stats, 20.0f);
    require(hud == "● LOD 32.0k/200.0k pts  ◇ chunks 6/40 visible 8 cache 5 queued 3  ◷ 50 fps  progressive",
            "HUD line should include visible, cached, and queued chunk counts when streaming stats are present");
}

void test_motion_state_labels_refinement() {
    AnimationState moving;
    moving.moving = true;
    moving.frame_index = 1;
    require(formatMotionState(moving) == "refining ◝", "Moving state should show animated refinement");

    AnimationState stable;
    stable.moving = false;
    stable.frame_index = 4;
    require(formatMotionState(stable) == "stable ◆", "Stable state should show settled marker");
}

void test_progress_pulse_cycles_smoothly() {
    require(progressPulse(0) == "◜", "Pulse frame 0 should match spinner sequence");
    require(progressPulse(1) == "◝", "Pulse frame 1 should match spinner sequence");
    require(progressPulse(4) == "◜", "Pulse should wrap by frame index");
}

void test_terminal_restore_sequence_disables_mouse_reporting() {
    const auto sequence = terminalRestoreSequence();
    require(sequence.find("\x1b[?1000l") != std::string::npos, "Restore sequence should disable X10 mouse reporting");
    require(sequence.find("\x1b[?1002l") != std::string::npos, "Restore sequence should disable button-event mouse reporting");
    require(sequence.find("\x1b[?1003l") != std::string::npos, "Restore sequence should disable any-event mouse reporting");
    require(sequence.find("\x1b[?1006l") != std::string::npos, "Restore sequence should disable SGR mouse reporting");
}

int main() {
    test_hud_line_is_compact_and_informative();
    test_hud_line_includes_streaming_stats_when_present();
    test_motion_state_labels_refinement();
    test_progress_pulse_cycles_smoothly();
    test_terminal_restore_sequence_disables_mouse_reporting();
    return 0;
}
