add_executable(fuse_cli fuse_cli/main.cpp)
target_link_libraries(fuse_cli PRIVATE fusion::io fusion_options)
