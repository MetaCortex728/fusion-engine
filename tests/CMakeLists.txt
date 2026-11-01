add_library(doctest INTERFACE)
target_include_directories(doctest SYSTEM INTERFACE
    ${PROJECT_SOURCE_DIR}/third_party/doctest)

# Test-only synthetic scene and virtual sensors used as ground truth.
# Not part of the fusion library or its public API.
add_library(fusion_test_support STATIC
    support/scene.cpp
    support/sensors.cpp)
target_include_directories(fusion_test_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fusion_test_support
    PUBLIC Eigen3::Eigen
    PRIVATE fusion_options)

add_executable(fusion_tests
    main.cpp
    test_version.cpp
    test_synthetic_scene.cpp
    test_geometry.cpp
    test_projection_scene.cpp
    test_depth_map.cpp)
target_link_libraries(fusion_tests PRIVATE
    fusion::fusion fusion_test_support doctest fusion_options)

target_compile_definitions(fusion_tests PRIVATE
    FUSION_EXPECTED_VERSION="${PROJECT_VERSION}")

add_test(NAME fusion_tests COMMAND fusion_tests)
