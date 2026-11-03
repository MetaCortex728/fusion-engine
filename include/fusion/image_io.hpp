// PNG reading and writing for camera images (part of the fusion_io library).
//
// RGB images are 8-bit PNGs; thermal images are 16-bit greyscale PNGs that
// hold the raw counts unchanged.
#pragma once

#include <stdexcept>
#include <string>

#include "fusion/thermal.hpp"

namespace fusion {

/// Thrown for an unreadable, invalid, or unexpected image file. The message
/// starts with the file path.
class ImageError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// 8-bit RGB image, row-major. The same layout as a rendered DisplayImage.
using RgbImage = DisplayImage;

/// Reads an 8-bit PNG (greyscale, palette, RGB, with or without alpha; also
/// greyscale or palette at 1, 2 or 4 bits) as RGB; alpha is dropped. Throws
/// ImageError for an unreadable or invalid file, or a 16-bit PNG.
RgbImage read_rgb_png(const std::string& path);

/// Reads a 16-bit greyscale PNG as raw counts, bit-exact. Throws ImageError
/// for an unreadable or invalid file, or any other colour type or bit depth.
RawThermalImage read_thermal_png(const std::string& path);

/// Writes an 8-bit RGB PNG (always RGB, 8 bits per channel). Throws ImageError on failure or for an image
/// whose pixel count differs from width * height.
void write_rgb_png(const std::string& path, const RgbImage& image);

/// Writes raw counts as a 16-bit greyscale PNG (always 16 bits). Throws ImageError on failure.
void write_thermal_png(const std::string& path, const RawThermalImage& image);

}  // namespace fusion
