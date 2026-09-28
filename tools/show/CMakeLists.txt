add_executable(show
    main.cc
    options.cc
    log_parser.cc
    trajectory.cc
    replay.cc
)
target_link_libraries(show
    PRIVATE
    coverage_path_planning::common
    coverage_path_planning::viz
    coverage_path_planning::log
    coverage_path_planning::yaml_cpp
    coverage_path_planning::threads
)
# Keep the existing build/tools/show executable path.
set_target_properties(show PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)
