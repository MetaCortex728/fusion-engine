add_executable(fuse_cli fuse_cli/main.cpp)
target_link_libraries(fuse_cli PRIVATE fusion::io fusion_options)

add_executable(thermal_export thermal_export/main.cpp)
target_link_libraries(thermal_export PRIVATE fusion::io fusion_options)

add_executable(fusion_probe fusion_probe/main.cpp)
target_link_libraries(fusion_probe PRIVATE fusion::io fusion_options)
