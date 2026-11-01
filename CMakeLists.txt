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

# Core library: no GUI or dataset dependencies.
add_library(fusion STATIC
    src/version.cpp)
add_library(fusion::fusion ALIAS fusion)
target_include_directories(fusion PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_definitions(fusion PRIVATE
    FUSION_VERSION_STRING="${PROJECT_VERSION}")
target_link_libraries(fusion PRIVATE $<BUILD_INTERFACE:fusion_options>)

if(FUSION_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
