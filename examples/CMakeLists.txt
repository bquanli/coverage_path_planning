add_executable(trajectory_show trajectory_show.cc)
target_link_libraries(trajectory_show
    PUBLIC
    coverage_path_planning::viz
)