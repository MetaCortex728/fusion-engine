#include "fusion/image_io.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

#include <lodepng.h>

namespace fusion {

namespace {

[[noreturn]] void fail(const std::string& path, const std::string& what) {
    throw ImageError(path + ": " + what);
}

std::vector<unsigned char> load_file(const std::string& path) {
    std::vector<unsigned char> bytes;
    if (lodepng::load_file(bytes, path) != 0) fail(path, "cannot read file");
    return bytes;
}

struct Header {
    unsigned width = 0;
    unsigned height = 0;
    LodePNGColorType colortype = LCT_GREY;
    unsigned bitdepth = 0;
};

// Reads the PNG header: size, colour type and bit depth.
Header inspect(const std::string& path, const std::vector<unsigned char>& bytes) {
    Header h;
    lodepng::State state;
    const unsigned err = lodepng_inspect(&h.width, &h.height, &state, bytes.data(), bytes.size());
    if (err != 0) fail(path, std::string("not a valid PNG: ") + lodepng_error_text(err));
    h.colortype = state.info_png.color.colortype;
    h.bitdepth = state.info_png.color.bitdepth;
    return h;
}

// Encodes exactly in the given colour type and bit depth: lodepng would
// otherwise pick the smallest type that holds the pixels (e.g. a palette, or
// 8 bits for 16-bit values whose two bytes are equal).
void encode_exact(const std::string& path, const std::vector<unsigned char>& pixels, unsigned w,
                  unsigned h, LodePNGColorType type, unsigned bitdepth) {
    lodepng::State state;
    state.info_raw.colortype = type;
    state.info_raw.bitdepth = bitdepth;
    state.info_png.color.colortype = type;
    state.info_png.color.bitdepth = bitdepth;
    state.encoder.auto_convert = 0;
    std::vector<unsigned char> png;
    unsigned err = lodepng::encode(png, pixels, w, h, state);
    if (err == 0) err = lodepng::save_file(png, path);
    if (err != 0) fail(path, std::string("cannot write PNG: ") + lodepng_error_text(err));
}

int checked_size(const std::string& path, unsigned v) {
    if (v == 0 || v > (1u << 20)) fail(path, "image size out of range");
    return static_cast<int>(v);
}

}  // namespace

RgbImage read_rgb_png(const std::string& path) {
    const std::vector<unsigned char> bytes = load_file(path);
    const Header head = inspect(path, bytes);
    if (head.bitdepth > 8) {
        fail(path, "expected an 8-bit PNG for an RGB image, got bit depth " +
                       std::to_string(head.bitdepth));
    }
    unsigned w = 0, h = 0;
    std::vector<unsigned char> rgb;
    const unsigned err = lodepng::decode(rgb, w, h, bytes, LCT_RGB, 8);
    if (err != 0) fail(path, std::string("cannot decode PNG: ") + lodepng_error_text(err));
    RgbImage img;
    img.width = checked_size(path, w);
    img.height = checked_size(path, h);
    img.pixels.resize(static_cast<std::size_t>(w) * h);
    for (std::size_t i = 0; i < img.pixels.size(); ++i) {
        img.pixels[i] = {rgb[3 * i], rgb[3 * i + 1], rgb[3 * i + 2]};
    }
    return img;
}

RawThermalImage read_thermal_png(const std::string& path) {
    const std::vector<unsigned char> bytes = load_file(path);
    const Header head = inspect(path, bytes);
    if (head.colortype != LCT_GREY || head.bitdepth != 16) {
        fail(path, "expected a 16-bit greyscale PNG of raw thermal counts, got colour type " +
                       std::to_string(static_cast<int>(head.colortype)) + " with bit depth " +
                       std::to_string(head.bitdepth));
    }
    unsigned w = 0, h = 0;
    std::vector<unsigned char> raw;
    const unsigned err = lodepng::decode(raw, w, h, bytes, LCT_GREY, 16);
    if (err != 0) fail(path, std::string("cannot decode PNG: ") + lodepng_error_text(err));
    std::vector<std::uint16_t> values(static_cast<std::size_t>(w) * h);
    for (std::size_t i = 0; i < values.size(); ++i) {
        // PNG stores 16-bit samples big-endian.
        values[i] = static_cast<std::uint16_t>((raw[2 * i] << 8) | raw[2 * i + 1]);
    }
    return RawThermalImage(checked_size(path, w), checked_size(path, h), std::move(values));
}

void write_rgb_png(const std::string& path, const RgbImage& image) {
    if (image.width <= 0 || image.height <= 0 ||
        image.pixels.size() != static_cast<std::size_t>(image.width) *
                                   static_cast<std::size_t>(image.height)) {
        fail(path, "image size does not match its pixel count");
    }
    std::vector<unsigned char> rgb(image.pixels.size() * 3);
    for (std::size_t i = 0; i < image.pixels.size(); ++i) {
        rgb[3 * i] = image.pixels[i].r;
        rgb[3 * i + 1] = image.pixels[i].g;
        rgb[3 * i + 2] = image.pixels[i].b;
    }
    encode_exact(path, rgb, static_cast<unsigned>(image.width), static_cast<unsigned>(image.height),
                 LCT_RGB, 8);
}

void write_thermal_png(const std::string& path, const RawThermalImage& image) {
    const std::vector<std::uint16_t>& values = image.values();
    std::vector<unsigned char> raw(values.size() * 2);
    for (std::size_t i = 0; i < values.size(); ++i) {
        raw[2 * i] = static_cast<unsigned char>(values[i] >> 8);
        raw[2 * i + 1] = static_cast<unsigned char>(values[i] & 0xFF);
    }
    encode_exact(path, raw, static_cast<unsigned>(image.width()),
                 static_cast<unsigned>(image.height()), LCT_GREY, 16);
}

}  // namespace fusion
