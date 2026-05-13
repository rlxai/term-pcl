#include "term_pcl/color_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace term_pcl {
namespace {

float normalized(float value, float min_value, float max_value) {
    if (max_value <= min_value) return 0.0f;
    return std::clamp((value - min_value) / (max_value - min_value), 0.0f, 1.0f);
}

ftxui::Color rgb(unsigned char r, unsigned char g, unsigned char b) {
    return ftxui::Color::RGB(r, g, b);
}

ftxui::Color rainbow(float v) {
    unsigned char r = 0, g = 0, b = 0;
    if (v < 0.25f) { r = 255; g = static_cast<unsigned char>(v * 1020); }
    else if (v < 0.5f) { r = static_cast<unsigned char>((0.5f - v) * 1020); g = 255; }
    else if (v < 0.75f) { g = 255; b = static_cast<unsigned char>((v - 0.5f) * 1020); }
    else { g = static_cast<unsigned char>((1.0f - v) * 1020); b = 255; }
    return rgb(r, g, b);
}

ftxui::Color grayscale(float v) {
    const auto c = static_cast<unsigned char>(std::round(v * 255.0f));
    return rgb(c, c, c);
}

ftxui::Color heat(float v) {
    const auto r = static_cast<unsigned char>(std::round(std::min(1.0f, v * 2.0f) * 255.0f));
    const auto g = static_cast<unsigned char>(std::round(std::clamp((v - 0.35f) * 2.0f, 0.0f, 1.0f) * 255.0f));
    const auto b = static_cast<unsigned char>(std::round(std::clamp((v - 0.75f) * 4.0f, 0.0f, 1.0f) * 255.0f));
    return rgb(r, g, b);
}

ftxui::Color viridis(float v) {
    const unsigned char stops[][3] = {{68, 1, 84}, {59, 82, 139}, {33, 145, 140}, {94, 201, 98}, {253, 231, 37}};
    const float scaled = v * 4.0f;
    const int i = std::min(3, static_cast<int>(scaled));
    const float t = scaled - static_cast<float>(i);
    return rgb(static_cast<unsigned char>(stops[i][0] + (stops[i + 1][0] - stops[i][0]) * t),
               static_cast<unsigned char>(stops[i][1] + (stops[i + 1][1] - stops[i][1]) * t),
               static_cast<unsigned char>(stops[i][2] + (stops[i + 1][2] - stops[i][2]) * t));
}

ftxui::Color turbo(float v) {
    const unsigned char stops[][3] = {{48, 18, 59}, {50, 101, 255}, {45, 200, 130}, {245, 210, 70}, {180, 4, 38}};
    const float scaled = v * 4.0f;
    const int i = std::min(3, static_cast<int>(scaled));
    const float t = scaled - static_cast<float>(i);
    return rgb(static_cast<unsigned char>(stops[i][0] + (stops[i + 1][0] - stops[i][0]) * t),
               static_cast<unsigned char>(stops[i][1] + (stops[i + 1][1] - stops[i][1]) * t),
               static_cast<unsigned char>(stops[i][2] + (stops[i + 1][2] - stops[i][2]) * t));
}

ftxui::Color scalarColor(float v, ColorPalette palette) {
    switch (palette) {
        case ColorPalette::Rainbow: return rainbow(v);
        case ColorPalette::Turbo: return turbo(v);
        case ColorPalette::Viridis: return viridis(v);
        case ColorPalette::Heat: return heat(v);
        case ColorPalette::Grayscale: return grayscale(v);
    }
    return rainbow(v);
}

}  // namespace

ColorMode parseColorMode(const std::string& value) {
    if (value == "rgb") return ColorMode::Rgb;
    if (value == "elevation") return ColorMode::Elevation;
    if (value == "x") return ColorMode::X;
    if (value == "white") return ColorMode::White;
    if (value == "rainbow") return ColorMode::Rainbow;
    if (value == "turbo") return ColorMode::Turbo;
    if (value == "viridis") return ColorMode::Viridis;
    if (value == "heat") return ColorMode::Heat;
    if (value == "grayscale") return ColorMode::Grayscale;
    throw std::invalid_argument("Invalid --color: " + value);
}

std::string colorModeName(ColorMode mode) {
    switch (mode) {
        case ColorMode::Rgb: return "rgb";
        case ColorMode::Elevation: return "elevation";
        case ColorMode::X: return "x";
        case ColorMode::White: return "white";
        case ColorMode::Rainbow: return "rainbow";
        case ColorMode::Turbo: return "turbo";
        case ColorMode::Viridis: return "viridis";
        case ColorMode::Heat: return "heat";
        case ColorMode::Grayscale: return "grayscale";
    }
    return "rgb";
}

ColorMode nextColorMode(ColorMode mode) {
    switch (mode) {
        case ColorMode::Rgb: return ColorMode::Elevation;
        case ColorMode::Elevation: return ColorMode::X;
        case ColorMode::X: return ColorMode::White;
        case ColorMode::White: return ColorMode::Turbo;
        case ColorMode::Turbo: return ColorMode::Viridis;
        case ColorMode::Viridis: return ColorMode::Heat;
        case ColorMode::Heat: return ColorMode::Grayscale;
        case ColorMode::Grayscale: return ColorMode::Rainbow;
        case ColorMode::Rainbow: return ColorMode::Rgb;
    }
    return ColorMode::Rgb;
}

ftxui::Color colorPoint(const Point& point, const CloudData& cloud, const ColorSettings& settings) {
    if (settings.mode == ColorMode::Rgb && point.has_color) {
        return rgb(point.r, point.g, point.b);
    }
    if (settings.mode == ColorMode::White) {
        return ftxui::Color::White;
    }

    ColorPalette palette = settings.scalar_palette;
    float value = normalized(point.z, cloud.min_z, cloud.max_z);
    if (settings.mode == ColorMode::X) {
        value = normalized(point.x, cloud.min_x, cloud.max_x);
        palette = ColorPalette::Rainbow;
    } else if (settings.mode == ColorMode::Rainbow) {
        value = normalized(point.z, cloud.min_z, cloud.max_z);
        palette = ColorPalette::Rainbow;
    } else if (settings.mode == ColorMode::Turbo) {
        palette = ColorPalette::Turbo;
    } else if (settings.mode == ColorMode::Viridis) {
        palette = ColorPalette::Viridis;
    } else if (settings.mode == ColorMode::Heat) {
        palette = ColorPalette::Heat;
    } else if (settings.mode == ColorMode::Grayscale) {
        palette = ColorPalette::Grayscale;
    }
    return scalarColor(value, palette);
}

}  // namespace term_pcl
