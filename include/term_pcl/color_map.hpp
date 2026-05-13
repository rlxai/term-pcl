#ifndef TERMINAL_PCL_VISUALIZER_COLOR_MAP_HPP_
#define TERMINAL_PCL_VISUALIZER_COLOR_MAP_HPP_

#include "term_pcl/types.hpp"

#include "ftxui/screen/color.hpp"

#include <string>

namespace term_pcl {

enum class ColorMode {
    Rgb,
    Elevation,
    X,
    White,
    Rainbow,
    Turbo,
    Viridis,
    Heat,
    Grayscale
};

enum class ColorPalette {
    Rainbow,
    Turbo,
    Viridis,
    Heat,
    Grayscale
};

struct ColorSettings {
    ColorMode mode = ColorMode::Rgb;
    ColorPalette scalar_palette = ColorPalette::Rainbow;
};

ColorMode parseColorMode(const std::string& value);
std::string colorModeName(ColorMode mode);
ColorMode nextColorMode(ColorMode mode);
ftxui::Color colorPoint(const Point& point, const CloudData& cloud, const ColorSettings& settings);

}  // namespace term_pcl

#endif  // TERMINAL_PCL_VISUALIZER_COLOR_MAP_HPP_
