add_library(doctest INTERFACE)
target_include_directories(doctest SYSTEM INTERFACE
    ${PROJECT_SOURCE_DIR}/third_party/doctest)

add_executable(fusion_tests
    main.cpp
    test_version.cpp)
target_link_libraries(fusion_tests PRIVATE fusion::fusion doctest fusion_options)

target_compile_definitions(fusion_tests PRIVATE
    FUSION_EXPECTED_VERSION="${PROJECT_VERSION}")

add_test(NAME fusion_tests COMMAND fusion_tests)
