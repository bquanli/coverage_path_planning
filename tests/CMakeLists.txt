  add_executable(coverage_debug coverage_debug.cc)
  target_link_libraries(coverage_debug
      PRIVATE
      foxglove_cpp
  )