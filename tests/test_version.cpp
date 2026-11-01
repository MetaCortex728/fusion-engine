#include <doctest.h>

#include <string>

#include "fusion/version.hpp"

TEST_CASE("version() returns the project version") {
    const char* v = fusion::version();
    REQUIRE(v != nullptr);
    CHECK(std::string(v) == FUSION_EXPECTED_VERSION);
}
