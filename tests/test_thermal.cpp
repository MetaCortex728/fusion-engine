// Raw thermal images, Planck conversion, and window/level display.
#include <doctest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <fusion/thermal.hpp>

using fusion::LinearParams;
using fusion::PlanckParams;
using fusion::ThermalCalibration;
using fusion::RawThermalImage;
using fusion::Rgb8;
using fusion::WindowLevel;

namespace {

// Constants of configs/thermal_camera.yaml.
const PlanckParams kExample{380747.0, 1428.0, 1.0, -88.539};

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

}  // namespace

TEST_CASE("raw thermal values round-trip unchanged, including 0 and 65535") {
    const std::vector<std::uint16_t> values = {0, 1, 2, 255, 256, 32767, 32768, 65534, 65535,
                                               14000, 21000, 12345};
    const RawThermalImage image(4, 3, values);
    CHECK(image.width() == 4);
    CHECK(image.height() == 3);
    CHECK(image.values() == values);
    CHECK(image.at(0, 0) == 0);
    CHECK(image.at(0, 2) == 65535);  // index 8
    CHECK(image.at(3, 2) == 12345);

    RawThermalImage blank(2, 2);
    CHECK(blank.values() == std::vector<std::uint16_t>(4, 0));
    blank.set(1, 0, 65535);
    blank.set(0, 1, 0);
    blank.set(1, 1, 1);
    CHECK(blank.at(1, 0) == 65535);
    CHECK(blank.at(0, 1) == 0);
    CHECK(blank.at(1, 1) == 1);

    // Converting and rendering for display only reads the raw values.
    (void)fusion::render_display(image, kExample, WindowLevel{30.0, 40.0});
    for (const std::uint16_t s : values) (void)fusion::raw_to_celsius(s, kExample);
    CHECK(image.values() == values);
}

TEST_CASE("raw thermal image rejects bad sizes and pixels") {
    CHECK_THROWS_AS(RawThermalImage(0, 3), std::invalid_argument);
    CHECK_THROWS_AS(RawThermalImage(2, -1), std::invalid_argument);
    CHECK_THROWS_AS(RawThermalImage(2, 2, std::vector<std::uint16_t>(3, 0)),
                    std::invalid_argument);
    RawThermalImage image(2, 2);
    CHECK_THROWS_AS((void)image.at(2, 0), std::out_of_range);
    CHECK_THROWS_AS((void)image.at(0, -1), std::out_of_range);
    CHECK_THROWS_AS(image.set(-1, 0, 5), std::out_of_range);
}

TEST_CASE("raw to Celsius matches independently computed values within 1e-6 degrees") {
    // Expected values computed separately with Python's decimal module at 50
    // significant digits from T = B / ln(R / (S - O) + F) - 273.15.
    struct Case {
        std::uint16_t raw;
        double celsius;
    };
    const Case example[] = {
        {0, -102.47297650423156162},
        {1, -102.24361110993418280},
        {1000, -29.470378725861270085},
        {7500, 89.725805297920267523},
        {15000, 163.93977699218147774},
        {30000, 273.12904477764036296},
        {65535, 471.68040978679333299},
    };
    for (const Case& c : example) {
        CAPTURE(c.raw);
        CHECK(std::abs(fusion::raw_to_celsius(c.raw, kExample) - c.celsius) <= 1e-6);
    }

    // R = 1000, B = 500, F = 0.5, O = 10. For S = 110 the argument is
    // 1000 / 100 + 0.5 = 10.5, so T = 500 / ln(10.5) - 273.15.
    const PlanckParams other{1000.0, 500.0, 0.5, 10.0};
    const Case other_cases[] = {
        {110, -60.508483731277156693},
        {60, -107.61037599092199807},
        {1010, 960.00173118821584300},
    };
    for (const Case& c : other_cases) {
        CAPTURE(c.raw);
        CHECK(std::abs(fusion::raw_to_celsius(c.raw, other) - c.celsius) <= 1e-6);
    }
}

TEST_CASE("raw to Celsius rejects inputs that make the formula undefined") {
    // S == O: division by zero.
    CHECK_THROWS_AS(fusion::raw_to_celsius(100, PlanckParams{380747.0, 1428.0, 1.0, 100.0}),
                    std::domain_error);
    // R / (S - O) + F < 0: 380747 / (500 - 1000) + 1 = -760.494.
    CHECK_THROWS_AS(fusion::raw_to_celsius(500, PlanckParams{380747.0, 1428.0, 1.0, 1000.0}),
                    std::domain_error);
    // R / (S - O) + F == 0: 100 / (0 - 100) + 1 = 0.
    CHECK_THROWS_AS(fusion::raw_to_celsius(0, PlanckParams{100.0, 1428.0, 1.0, 100.0}),
                    std::domain_error);
    // R / (S - O) + F == 1, so ln() == 0: 100 / 200 + 0.5 = 1.
    CHECK_THROWS_AS(fusion::raw_to_celsius(200, PlanckParams{100.0, 1428.0, 0.5, 0.0}),
                    std::domain_error);
    // Argument in (0, 1) gives a negative Kelvin temperature: 100 / 400 + 0.5.
    CHECK_THROWS_AS(fusion::raw_to_celsius(400, PlanckParams{100.0, 1428.0, 0.5, 0.0}),
                    std::domain_error);

    // Invalid constants.
    CHECK_THROWS_AS(fusion::raw_to_celsius(1000, PlanckParams{0.0, 1428.0, 1.0, 0.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(1000, PlanckParams{-5.0, 1428.0, 1.0, 0.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(1000, PlanckParams{380747.0, 0.0, 1.0, 0.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(1000, PlanckParams{kNaN, 1428.0, 1.0, 0.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(1000, PlanckParams{380747.0, 1428.0, kInf, 0.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(1000, PlanckParams{380747.0, 1428.0, 1.0, kNaN}),
                    std::invalid_argument);
}

TEST_CASE("window/level maps the endpoints, centre, and clamps outside the window") {
    const WindowLevel w{30.0, 20.0};  // window spans [20, 40] degrees C
    CHECK(fusion::window_level(20.0, w) == 0.0);
    CHECK(fusion::window_level(40.0, w) == 1.0);
    CHECK(fusion::window_level(30.0, w) == 0.5);
    CHECK(fusion::window_level(25.0, w) == 0.25);
    CHECK(fusion::window_level(19.999, w) == 0.0);
    CHECK(fusion::window_level(-273.0, w) == 0.0);
    CHECK(fusion::window_level(40.001, w) == 1.0);
    CHECK(fusion::window_level(1e6, w) == 1.0);
}

TEST_CASE("window/level rejects a non-positive or non-finite window") {
    CHECK_THROWS_AS(fusion::window_level(30.0, WindowLevel{30.0, 0.0}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::window_level(30.0, WindowLevel{30.0, -1.0}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::window_level(30.0, WindowLevel{30.0, kNaN}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::window_level(30.0, WindowLevel{kInf, 10.0}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::window_level(kNaN, WindowLevel{30.0, 10.0}), std::invalid_argument);
    const RawThermalImage image(2, 2);
    CHECK_THROWS_AS(fusion::render_display(image, kExample, WindowLevel{30.0, 0.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(fusion::render_display(image, kExample, WindowLevel{30.0, -5.0}),
                    std::invalid_argument);
}

TEST_CASE("hot colormap goes from black through red and yellow to white") {
    CHECK(fusion::hot_colormap(0.0) == Rgb8{0, 0, 0});
    CHECK(fusion::hot_colormap(1.0 / 3.0) == Rgb8{255, 0, 0});
    CHECK(fusion::hot_colormap(0.5) == Rgb8{255, 128, 0});  // round(127.5)
    CHECK(fusion::hot_colormap(2.0 / 3.0) == Rgb8{255, 255, 0});
    CHECK(fusion::hot_colormap(1.0) == Rgb8{255, 255, 255});
    CHECK(fusion::hot_colormap(-0.5) == Rgb8{0, 0, 0});
    CHECK(fusion::hot_colormap(1.5) == Rgb8{255, 255, 255});
    CHECK_THROWS_AS(fusion::hot_colormap(kNaN), std::invalid_argument);
}

TEST_CASE("display image applies conversion, window/level, and colormap per pixel") {
    // 7500 -> 89.73 C, 15000 -> 163.94 C, 30000 -> 273.13 C. With the window
    // [90, 270], the first clamps to black and the last to white.
    const RawThermalImage image(3, 1, {7500, 15000, 30000});
    const WindowLevel w{180.0, 180.0};
    const auto display = fusion::render_display(image, kExample, w);
    CHECK(display.width == 3);
    CHECK(display.height == 1);
    REQUIRE(display.pixels.size() == 3);
    CHECK(display.at(0, 0) == Rgb8{0, 0, 0});
    CHECK(display.at(1, 0) ==
          fusion::hot_colormap(fusion::window_level(fusion::raw_to_celsius(15000, kExample), w)));
    CHECK(display.at(2, 0) == Rgb8{255, 255, 255});
    CHECK(image.values() == std::vector<std::uint16_t>{7500, 15000, 30000});

    // A pixel whose raw value equals O cannot be converted and is drawn black.
    const RawThermalImage bad(2, 1, {100, 30000});
    const auto bad_display =
        fusion::render_display(bad, PlanckParams{380747.0, 1428.0, 1.0, 100.0}, w);
    CHECK(bad_display.at(0, 0) == Rgb8{0, 0, 0});
    CHECK(bad_display.at(1, 0) == Rgb8{255, 255, 255});
}

TEST_CASE("linear encoding: T = raw * scale + offset, exact for binary-exact parameters") {
    // 0.0625 and -40 are exact in binary, so every product and sum is exact.
    const LinearParams p{0.0625, -40.0};
    CHECK(fusion::raw_to_celsius(0, p) == -40.0);
    CHECK(fusion::raw_to_celsius(1, p) == -39.9375);
    CHECK(fusion::raw_to_celsius(1000, p) == 22.5);
    CHECK(fusion::raw_to_celsius(32768, p) == 2008.0);
    CHECK(fusion::raw_to_celsius(65535, p) == 4055.9375);

    // A negative scale is allowed.
    const LinearParams neg{-0.5, 100.0};
    CHECK(fusion::raw_to_celsius(0, neg) == 100.0);
    CHECK(fusion::raw_to_celsius(65535, neg) == -32667.5);

    // Offset only shifts; scale 1 passes raw values through.
    CHECK(fusion::raw_to_celsius(65535, LinearParams{1.0, 0.0}) == 65535.0);
}

TEST_CASE("linear encoding in centi-kelvin") {
    const LinearParams ck{0.01, -273.15};
    // 0 * 0.01 is exactly 0, so raw 0 gives the offset exactly.
    CHECK(fusion::raw_to_celsius(0, ck) == -273.15);
    // 0.01 is not exact in binary; the result is within one rounding.
    CHECK(std::abs(fusion::raw_to_celsius(65535, ck) - 382.2) <= 1e-12);
    CHECK(std::abs(fusion::raw_to_celsius(27315, ck) - 0.0) <= 1e-12);
    CHECK(std::abs(fusion::raw_to_celsius(29315, ck) - 20.0) <= 1e-12);
    CHECK(std::abs(fusion::raw_to_celsius(25315, ck) - -20.0) <= 1e-12);
    // Same arithmetic as the formula, so equal bit for bit.
    CHECK(fusion::raw_to_celsius(65535, ck) == 65535.0 * 0.01 - 273.15);
}

TEST_CASE("linear encoding rejects non-finite parameters and a zero scale") {
    CHECK_NOTHROW(fusion::validate(LinearParams{0.01, -273.15}));
    CHECK_THROWS_AS(fusion::validate(LinearParams{0.0, -273.15}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::validate(LinearParams{-0.0, 0.0}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(100, LinearParams{0.0, 1.0}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(100, LinearParams{kNaN, 1.0}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(100, LinearParams{kInf, 1.0}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(100, LinearParams{-kInf, 1.0}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(100, LinearParams{0.01, kNaN}), std::invalid_argument);
    CHECK_THROWS_AS(fusion::raw_to_celsius(100, LinearParams{0.01, kInf}), std::invalid_argument);
    const RawThermalImage image(2, 2);
    CHECK_THROWS_AS(fusion::render_display(image, ThermalCalibration::from_linear({0.0, 0.0}),
                                           WindowLevel{30.0, 10.0}),
                    std::invalid_argument);
}

TEST_CASE("thermal calibration dispatches on the encoding") {
    const ThermalCalibration planck = ThermalCalibration::from_planck(kExample);
    const ThermalCalibration linear = ThermalCalibration::from_linear({0.0625, -40.0});
    CHECK(planck.encoding == fusion::ThermalEncoding::planck);
    CHECK(linear.encoding == fusion::ThermalEncoding::linear);
    for (const std::uint16_t s : {std::uint16_t{1}, std::uint16_t{14000}, std::uint16_t{65535}}) {
        CHECK(fusion::raw_to_celsius(s, planck) == fusion::raw_to_celsius(s, kExample));
    }
    CHECK(fusion::raw_to_celsius(0, linear) == -40.0);
    CHECK(fusion::raw_to_celsius(65535, linear) == 4055.9375);

    // A default calibration is Planck with zero constants, which is invalid.
    CHECK_THROWS_AS(fusion::validate(ThermalCalibration{}), std::invalid_argument);
    // Only the selected encoding's parameters are validated.
    ThermalCalibration mixed = linear;
    mixed.planck = PlanckParams{};  // invalid, but unused
    CHECK_NOTHROW(fusion::validate(mixed));
    CHECK_THROWS_AS(fusion::raw_to_celsius(100, ThermalCalibration::from_planck({})),
                    std::invalid_argument);
}

TEST_CASE("display image with the linear encoding") {
    // Centi-kelvin: 25315 -> -20 C, 27315 -> 0 C, 29315 -> 20 C, 0 -> -273.15 C,
    // 65535 -> 382.2 C. Window [-20, 20].
    const RawThermalImage image(5, 1, {25315, 27315, 29315, 0, 65535});
    const ThermalCalibration ck = ThermalCalibration::from_linear({0.01, -273.15});
    const WindowLevel w{0.0, 40.0};
    const auto display = fusion::render_display(image, ck, w);
    REQUIRE(display.pixels.size() == 5);
    CHECK(display.at(0, 0) == Rgb8{0, 0, 0});
    CHECK(display.at(1, 0) == fusion::hot_colormap(0.5));
    CHECK(display.at(1, 0) == Rgb8{255, 128, 0});
    CHECK(display.at(2, 0) == Rgb8{255, 255, 255});
    CHECK(display.at(3, 0) == Rgb8{0, 0, 0});        // clamps low
    CHECK(display.at(4, 0) == Rgb8{255, 255, 255});  // clamps high
    CHECK(image.values() == std::vector<std::uint16_t>{25315, 27315, 29315, 0, 65535});

    // The Planck overload is the Planck calibration.
    const RawThermalImage counts(3, 1, {7500, 15000, 30000});
    const WindowLevel w2{180.0, 180.0};
    CHECK(fusion::render_display(counts, kExample, w2).pixels ==
          fusion::render_display(counts, ThermalCalibration::from_planck(kExample), w2).pixels);
}
