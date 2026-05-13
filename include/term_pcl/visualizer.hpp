/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *  Copyright (c) 2026, Robolabs AI.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Robolabs AI nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef TERMINAL_PCL_VISUALIZER_VISUALIZER_HPP_
#define TERMINAL_PCL_VISUALIZER_VISUALIZER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "term_pcl/cloud_provider.hpp"
#include "term_pcl/color_map.hpp"

namespace term_pcl {

class Visualizer {
public:
    explicit Visualizer(std::shared_ptr<CloudProvider> cloud_provider, ColorSettings color_settings = {});
    void run();
    void stop();

private:
    std::shared_ptr<CloudProvider> cloud_provider_;
    ColorSettings color_settings_;
    ftxui::ScreenInteractive screen_;
    std::atomic<bool> quit_flag_{false};

    const float DEF_YAW = -1.5f;
    const float DEF_PITCH = 0.0f;
    const float DEF_ROLL = 1.5f;
    const float DEF_DIST = 5.0f;

    float cur_yaw_ = DEF_YAW;
    float cur_pitch_ = DEF_PITCH;
    float cur_roll_ = DEF_ROLL;
    float cur_dist_ = DEF_DIST;

    std::atomic<float> tar_yaw_{DEF_YAW};
    std::atomic<float> tar_pitch_{DEF_PITCH};
    std::atomic<float> tar_roll_{DEF_ROLL};
    std::atomic<float> tar_dist_{DEF_DIST};
    
    std::atomic<float> cam_x_{0.0f};
    std::atomic<float> cam_y_{0.0f};
    std::atomic<float> cam_z_{0.0f};
    
    std::atomic<float> zoom_{350.0f};
    std::atomic<bool> cam_mode_{true};

    float splat_multiplier_ = 1.0f;
    std::size_t frame_index_ = 0;
    float last_frame_ms_ = 0.0f;

    std::vector<float> z_buffer_;

    ftxui::Element render_frame();
    bool handle_event(ftxui::Event event);
};

} // namespace term_pcl

#endif // TERMINAL_PCL_VISUALIZER_VISUALIZER_HPP_
