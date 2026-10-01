add_executable(rtrajectory_show trajectory_show.cc)
target_link_libraries(rtrajectory_show
    PRIVATE
    coverage_path_planning::rerun
)
add_executable(simu_show simu_show.cc)
target_link_libraries(simu_show
    PRIVATE
    coverage_path_planning::rerun
)
add_executable(reference_line reference_line.cc)
target_link_libraries(reference_line
    PRIVATE
    coverage_path_planning::rerun
)