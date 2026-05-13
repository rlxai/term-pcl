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

#include "term_pcl/visualizer.hpp"
#include "term_pcl/color_map.hpp"
#include "term_pcl/terminal_ui.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "ftxui/screen/terminal.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/dom/canvas.hpp"

using namespace ftxui;
using namespace std::chrono_literals;

namespace term_pcl {

Visualizer::Visualizer(std::shared_ptr<CloudProvider> cloud_provider, ColorSettings color_settings)
    : cloud_provider_(std::move(cloud_provider)),
      color_settings_(color_settings),
      screen_(ScreenInteractive::TerminalOutput()) {}

void Visualizer::run() {
    std::cout << terminalRestoreSequence() << std::flush;
    auto renderer = Renderer([this]() { return render_frame(); });
    auto component = CatchEvent(renderer, [this](Event event) { return handle_event(event); });

    std::thread ui_thread([&]() {
        while (!quit_flag_) {
            screen_.PostEvent(Event::Custom);
            std::this_thread::sleep_for(30ms);
        }
        screen_.Exit();
    });

    screen_.Loop(component);
    quit_flag_ = true;
    if (ui_thread.joinable()) ui_thread.join();
    std::cout << terminalRestoreSequence() << std::flush;
}

void Visualizer::stop() {
    quit_flag_ = true;
    screen_.Exit();
    std::cout << terminalRestoreSequence() << std::flush;
}

Element Visualizer::render_frame() {
    const auto frame_start = std::chrono::steady_clock::now();
    auto terminal = Terminal::Size();

    // Reserve more height for stability (header=1, sep=1, footer=1, borders=4)
    const int target_height = std::max(10, terminal.dimy - 9);
    const int target_width = std::max(10, terminal.dimx - 4);

    const int sw = target_width * 2;
    const int sh = target_height * 4;

    cur_yaw_ += (tar_yaw_.load() - cur_yaw_) * 0.2f;
    cur_pitch_ += (tar_pitch_.load() - cur_pitch_) * 0.2f;
    cur_roll_ += (tar_roll_.load() - cur_roll_) * 0.2f;
    cur_dist_ += (tar_dist_.load() - cur_dist_) * 0.2f;

    const bool moving = std::abs(tar_yaw_.load() - cur_yaw_) > 0.001f ||
                        std::abs(tar_pitch_.load() - cur_pitch_) > 0.001f ||
                        std::abs(tar_roll_.load() - cur_roll_) > 0.001f ||
                        std::abs(tar_dist_.load() - cur_dist_) > 0.001f;

    RenderRequest request;
    request.screen_width = sw;
    request.screen_height = sh;
    request.point_budget = static_cast<std::size_t>(sw) * static_cast<std::size_t>(sh) / 2;
    request.yaw = cur_yaw_;
    request.pitch = cur_pitch_;
    request.roll = cur_roll_;
    request.zoom = zoom_.load();
    request.pan_x = cam_x_.load();
    request.pan_y = cam_y_.load();
    request.forward = cam_z_.load();
    request.moving = moving;
    request.frame_index = frame_index_++;

    auto data = cloud_provider_->get_data(request);
    const auto stats = cloud_provider_->stats();

    auto c = Canvas(sw, sh);
    int cx = sw / 2;
    int cy = sh / 2;

    if (z_buffer_.size() != static_cast<size_t>(sw * sh)) {
        z_buffer_.assign(sw * sh, 1000.0f);
    } else {
        std::fill(z_buffer_.begin(), z_buffer_.end(), 1000.0f);
    }
    float* z_ptr = z_buffer_.data();

    float czoom = zoom_.load();
    bool cm = cam_mode_.load();

    const float s_yaw = std::sin(cur_yaw_);
    const float c_yaw = std::cos(cur_yaw_);
    const float s_pitch = std::sin(cur_pitch_);
    const float c_pitch = std::cos(cur_pitch_);
    const float s_roll = std::sin(cur_roll_);
    const float c_roll = std::cos(cur_roll_);

    float cx_off = cam_x_.load();
    float cy_off = cam_y_.load();
    float cz_off = cam_z_.load();

    auto project = [&](float dx, float dy, float dz, int& out_sx, int& out_sy, float& out_z) {
        float rx, ry, rz;
        float tx = dx - cx_off;
        float ty = dy - cy_off;
        float tz = dz - cz_off;

        if (cm) { rx = tx; ry = ty; rz = tz; }
        else { rx = -ty; ry = -tz; rz = tx; }

        float x1 = rx * c_yaw + rz * s_yaw;
        float z1 = -rx * s_yaw + rz * c_yaw;
        float y2 = ry * c_pitch - z1 * s_pitch;
        float z2 = ry * s_pitch + z1 * c_pitch;
        float x3 = x1 * c_roll - y2 * s_roll;
        float y3 = x1 * s_roll + y2 * c_roll;

        out_z = z2 + cur_dist_;
        if (out_z > 0.1f) {
            float z_inv = 1.0f / out_z;
            out_sx = cx + static_cast<int>(czoom * x3 * z_inv);
            out_sy = cy + static_cast<int>(czoom * y3 * z_inv);
            return true;
        }
        return false;
    };

    auto z_plot_inline = [&](int x, int y, float sz, Color col) {
        if (x < 0 || x >= sw || y < 0 || y >= sh) return;
        int idx = y * sw + x;
        if (sz < z_ptr[idx]) {
            z_ptr[idx] = sz;
            c.DrawPoint(x, y, true, col);
        }
    };

    float floor_val = data->min_z - data->cz;
    for (float g = -4.0f; g <= 4.0f; g += 1.0f) {
        for (float t = -4.0f; t <= 4.0f; t += 0.2f) {
            int sx, sy; float sz;
            if (project(t, g, floor_val, sx, sy, sz)) z_plot_inline(sx, sy, sz, Color::GrayDark);
            if (project(g, t, floor_val, sx, sy, sz)) z_plot_inline(sx, sy, sz, Color::GrayDark);
        }
    }

    if (!data->points.empty()) {
        for (const auto& p : data->points) {
            int sx, sy; float sz;
            if (project(p.x - data->cx, p.y - data->cy, p.z - data->cz, sx, sy, sz)) {
                int r = 0;
                float base_r = (2.0f / (sz + 0.1f)) * splat_multiplier_;
                if (base_r > 2.0f) r = 2;      
                else if (base_r > 0.8f) r = 1; 
                else r = 0;                    

                Color col = colorPoint(p, *data, color_settings_);

                if (r == 0) {
                    z_plot_inline(sx, sy, sz, col);
                } else {
                    for (int dy = -r; dy <= r; ++dy) {
                        int py = sy + dy;
                        if (py < 0 || py >= sh) continue;
                        int row_off = py * sw;
                        for (int dx = -r; dx <= r; ++dx) {
                            int px = sx + dx;
                            if (px < 0 || px >= sw) continue;
                            int idx = row_off + px;
                            if (sz < z_ptr[idx]) {
                                z_ptr[idx] = sz;
                                c.DrawPoint(px, py, true, col);
                            }
                        }
                    }
                }
            }
        }
    }

    if (!data->points.empty()) {
        int sx, sy; float sz;
        if (project(data->min_x-data->cx, 0.0f, data->min_z - data->cz, sx, sy, sz)) {
            for (int i = -4; i <= 4; ++i) {
                z_plot_inline(sx + i, sy, sz, Color::White);
                z_plot_inline(sx, sy + i, sz, Color::White);
            }
        }
    }

    const auto frame_end = std::chrono::steady_clock::now();
    last_frame_ms_ = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start).count()) / 1000.0f;
    AnimationState animation_state;
    animation_state.moving = moving;
    animation_state.frame_index = frame_index_;
    const auto hud = formatHudLine(stats, last_frame_ms_);
    const auto motion_state = formatMotionState(animation_state);

    return vbox({
        hbox({
            text(" term-pcl ") | bold | color(Color::Yellow),
            separator(),
            text(" File: " + data->frame_id) | color(Color::Cyan),
            separator(),
            text(" Color: " + colorModeName(color_settings_.mode)) | color(Color::Cyan),
            filler(),
            text(cm ? "[CAMERA]" : "[ALT VIEW]") | bold | color(Color::Magenta),
            filler(),
            text(" Pts: " + std::to_string(data->points.size())) | color(Color::Green)
        }),
        hbox({
            text(" " + hud) | color(Color::Green),
            filler(),
            text(" " + motion_state + " ") | color(moving ? Color::Yellow : Color::Cyan)
        }),
        separator(),
        canvas(std::move(c)) | hcenter | flex | border,
        hbox({
            text(" [WASD] Orbit | [ARROWS] Pan | [1-3] Presets | [C] Camera | [M] Color | [[] Splat- | []] Splat+ | [Q] Quit ") | dim,
            filler(),
            text(quit_flag_ ? " EXITING... " : "") | bold | color(Color::Red)
        })
    }) | border;
}

bool Visualizer::handle_event(Event event) {
    if (event == Event::Character('q') || event == Event::Escape) { quit_flag_ = true; screen_.Exit(); return true; }

    // Camera Panning/Translation
    if (event == Event::ArrowUp)    { cam_z_ = cam_z_.load() + 0.2f; return true; }
    if (event == Event::ArrowDown)  { cam_z_ = cam_z_.load() - 0.2f; return true; }
    if (event == Event::ArrowLeft)  { cam_y_ = cam_y_.load() + 0.2f; return true; }
    if (event == Event::ArrowRight) { cam_y_ = cam_y_.load() - 0.2f; return true; }
    if (event == Event::PageUp)     { cam_x_ = cam_x_.load() + 0.2f; return true; }
    if (event == Event::PageDown)   { cam_x_ = cam_x_.load() - 0.2f; return true; }

    if (event == Event::Character('a')) { tar_pitch_ = tar_pitch_.load() - 0.2f; return true; }
    if (event == Event::Character('d')) { tar_pitch_ = tar_pitch_.load() + 0.2f; return true; }
    if (event == Event::Character('w')) { tar_yaw_ = tar_yaw_.load() - 0.2f; return true; }
    if (event == Event::Character('s')) { tar_yaw_ = tar_yaw_.load() + 0.2f; return true; }
    if (event == Event::Character('p')) { tar_roll_ = tar_roll_.load() + 0.2f; return true; }
    if (event == Event::Character(';')) { tar_roll_ = tar_roll_.load() - 0.2f; return true; }
    if (event == Event::Character('c')) { cam_mode_ = !cam_mode_.load(); return true; }
    if (event == Event::Character('m') || event == Event::Character('M')) { color_settings_.mode = nextColorMode(color_settings_.mode); return true; }

    if (event == Event::Character('1')) { tar_yaw_ = DEF_YAW; tar_pitch_ = 0.0f; tar_roll_ = DEF_ROLL; tar_dist_ = 5.0f; cam_x_ = 0.0f; cam_y_ = 0.0f; cam_z_ = 0.0f; return true; }
    if (event == Event::Character('2')) { tar_yaw_ = DEF_YAW; tar_pitch_ = 1.5f; tar_roll_ = DEF_ROLL; tar_dist_ = 8.0f; cam_x_ = 0.0f; cam_y_ = 0.0f; cam_z_ = 0.0f; return true; }
    if (event == Event::Character('3')) { tar_yaw_ = DEF_YAW - 1.5f; tar_pitch_ = 0.0f; tar_roll_ = DEF_ROLL; tar_dist_ = 8.0f; cam_x_ = 0.0f; cam_y_ = 0.0f; cam_z_ = 0.0f; return true; }
    
    if (event == Event::Character('r')) { tar_yaw_ = DEF_YAW; tar_pitch_ = DEF_PITCH; tar_roll_ = DEF_ROLL; tar_dist_ = DEF_DIST; cam_x_ = 0.0f; cam_y_ = 0.0f; cam_z_ = 0.0f; splat_multiplier_ = 1.0f; return true; }
    if (event == Event::Character('[')) { splat_multiplier_ = std::max(0.1f, splat_multiplier_ - 0.1f); return true; }
    if (event == Event::Character(']')) { splat_multiplier_ = std::min(5.0f, splat_multiplier_ + 0.1f); return true; }
    if (event == Event::Character('+') || event == Event::Character('=')) { tar_dist_ = tar_dist_.load() - 0.5f; return true; }
    if (event == Event::Character('-') || event == Event::Character('_')) { tar_dist_ = tar_dist_.load() + 0.5f; return true; }
    return false;
}

} // namespace term_pcl
