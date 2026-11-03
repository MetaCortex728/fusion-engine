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

# Synthetic sequence in the on-disk sequence format, and a tool that writes it.
add_library(fusion_test_sequence STATIC support/synthetic_sequence.cpp)
target_link_libraries(fusion_test_sequence
    PUBLIC fusion_test_support fusion::io
    PRIVATE fusion_options)

add_executable(make_synthetic_sequence tools/make_synthetic_sequence.cpp)
target_link_libraries(make_synthetic_sequence PRIVATE fusion_test_sequence fusion_options)

add_executable(fusion_tests
    main.cpp
    test_version.cpp
    test_synthetic_scene.cpp
    test_geometry.cpp
    test_projection_scene.cpp
    test_depth_map.cpp
    test_config.cpp
    test_thermal.cpp
    test_thermal_projection.cpp
    test_time.cpp
    test_pose.cpp
    test_lidar_compensation.cpp
    test_aligned_frame.cpp
    test_sequence.cpp)
target_link_libraries(fusion_tests PRIVATE
    fusion::fusion fusion::io fusion_test_support fusion_test_sequence doctest fusion_options)

# End-to-end test of fuse_cli, which it runs as a separate process.
if(TARGET fuse_cli)
    target_sources(fusion_tests PRIVATE test_fuse_cli.cpp)
    target_compile_definitions(fusion_tests PRIVATE FUSE_CLI_PATH="$<TARGET_FILE:fuse_cli>")
    add_dependencies(fusion_tests fuse_cli)
endif()
if(TARGET thermal_export)
    target_sources(fusion_tests PRIVATE test_thermal_export.cpp)
    target_compile_definitions(fusion_tests PRIVATE
        THERMAL_EXPORT_PATH="$<TARGET_FILE:thermal_export>")
    add_dependencies(fusion_tests thermal_export)
endif()

target_compile_definitions(fusion_tests PRIVATE
    FUSION_EXPECTED_VERSION="${PROJECT_VERSION}"
    FUSION_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    FUSION_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}")

add_test(NAME fusion_tests COMMAND fusion_tests)
