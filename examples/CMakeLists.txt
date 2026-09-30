add_executable(trajectory_show trajectory_show.cc)
target_link_libraries(trajectory_show
    PRIVATE
    show_replay
)

add_subdirectory(grip_map)
add_subdirectory(rerun)