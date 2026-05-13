#include "term_pcl/color_map.hpp"

#include <stdexcept>
#include <string>

using term_pcl::CloudData;
using term_pcl::ColorMode;
using term_pcl::ColorPalette;
using term_pcl::ColorSettings;
using term_pcl::Point;
using term_pcl::colorPoint;
using term_pcl::parseColorMode;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_rgb_mode_uses_stored_point_color() {
    CloudData cloud;
    cloud.min_x = 0.0f;
    cloud.max_x = 1.0f;
    cloud.min_z = 0.0f;
    cloud.max_z = 1.0f;
    Point point{0.5f, 0.0f, 0.5f};
    point.has_color = true;
    point.r = 37;
    point.g = 216;
    point.b = 101;

    ColorSettings settings;
    settings.mode = ColorMode::Rgb;
    const auto color = colorPoint(point, cloud, settings);
    require(color == ftxui::Color::RGB(37, 216, 101), "RGB mode should use stored point color");
}

void test_rgb_mode_falls_back_to_elevation_when_color_missing() {
    CloudData cloud;
    cloud.min_z = 0.0f;
    cloud.max_z = 10.0f;
    Point low{0.0f, 0.0f, 0.0f};
    Point high{0.0f, 0.0f, 10.0f};

    ColorSettings settings;
    settings.mode = ColorMode::Rgb;
    const auto low_color = colorPoint(low, cloud, settings);
    const auto high_color = colorPoint(high, cloud, settings);
    require(low_color != high_color, "RGB mode should fall back to scalar coloring when RGB is unavailable");
}

void test_parse_color_modes() {
    require(parseColorMode("rgb") == ColorMode::Rgb, "Should parse rgb mode");
    require(parseColorMode("elevation") == ColorMode::Elevation, "Should parse elevation mode");
    require(parseColorMode("x") == ColorMode::X, "Should parse x mode");
    require(parseColorMode("white") == ColorMode::White, "Should parse white mode");
    require(parseColorMode("turbo") == ColorMode::Turbo, "Should parse turbo mode");
}

int main() {
    test_rgb_mode_uses_stored_point_color();
    test_rgb_mode_falls_back_to_elevation_when_color_missing();
    test_parse_color_modes();
    return 0;
}
