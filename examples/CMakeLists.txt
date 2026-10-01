if(COVERAGE_WITH_FOXGLOVE)
  add_executable(trajectory_show trajectory_show.cc)
  target_link_libraries(trajectory_show
      PRIVATE
      show_replay
  )
endif()

add_subdirectory(grip_map)

if(COVERAGE_WITH_RERUN)
  add_subdirectory(rerun)
endif()
