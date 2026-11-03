cmake_minimum_required(VERSION 3.21)

project(fusion-engine
    VERSION 0.1.0
    DESCRIPTION "Lightweight multi-sensor time and pixel alignment"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(FUSION_BUILD_TESTS "Build the fusion unit tests" ON)
option(FUSION_BUILD_APPS "Build the command-line apps (fuse_cli)" ON)
option(FUSION_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(FUSION_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

# Shared warning and sanitizer settings, applied per target.
add_library(fusion_options INTERFACE)
if(MSVC)
    target_compile_options(fusion_options INTERFACE /W4)
    if(FUSION_WARNINGS_AS_ERRORS)
        target_compile_options(fusion_options INTERFACE /WX)
    endif()
else()
    target_compile_options(fusion_options INTERFACE
        -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
    if(FUSION_WARNINGS_AS_ERRORS)
        target_compile_options(fusion_options INTERFACE -Werror)
    endif()
    if(FUSION_SANITIZE)
        target_compile_options(fusion_options INTERFACE
            -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
        target_link_options(fusion_options INTERFACE -fsanitize=address,undefined)
    endif()
endif()

# Third-party dependencies, found on the system and never vendored.
# Eigen is part of the public API (headers use Eigen types). yaml-cpp is an
# implementation detail of the config loader.
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
# yaml-cpp 0.8 exports yaml-cpp::yaml-cpp; 0.7 (Ubuntu 22.04) exports yaml-cpp.
if(TARGET yaml-cpp::yaml-cpp)
    set(FUSION_YAML_TARGET yaml-cpp::yaml-cpp)
else()
    set(FUSION_YAML_TARGET yaml-cpp)
endif()

# Core library: no GUI or dataset dependencies.
add_library(fusion STATIC
    src/aligned_frame.cpp
    src/config.cpp
    src/depth_map.cpp
    src/geometry.cpp
    src/lidar.cpp
    src/pose.cpp
    src/thermal.cpp
    src/time.cpp
    src/version.cpp)
add_library(fusion::fusion ALIAS fusion)
target_include_directories(fusion PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_definitions(fusion PRIVATE
    FUSION_VERSION_STRING="${PROJECT_VERSION}")
target_link_libraries(fusion
    PUBLIC Eigen3::Eigen
    PRIVATE ${FUSION_YAML_TARGET} $<BUILD_INTERFACE:fusion_options>)

# Vendored PNG codec (lodepng, zlib licence; see THIRD_PARTY_NOTICES.md).
# Built without the project's warning flags: it is third-party code.
add_library(fusion_lodepng STATIC third_party/lodepng/lodepng.cpp)
target_include_directories(fusion_lodepng SYSTEM PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/lodepng>)

# Sequence I/O: the on-disk sequence format and PNG images. Kept out of the
# core library so the core depends only on the standard library, Eigen, and
# yaml-cpp.
add_library(fusion_io STATIC
    src/io/image_io.cpp
    src/io/sequence.cpp)
add_library(fusion::io ALIAS fusion_io)
target_link_libraries(fusion_io
    PUBLIC fusion
    PRIVATE ${FUSION_YAML_TARGET} fusion_lodepng $<BUILD_INTERFACE:fusion_options>)

if(FUSION_BUILD_APPS)
    add_subdirectory(apps)
endif()

if(FUSION_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
