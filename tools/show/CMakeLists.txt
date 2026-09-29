find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(show_trajectory STATIC trajectory.cc)
target_link_libraries(show_trajectory PUBLIC coverage_path_planning::common)
target_include_directories(show_trajectory PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(show_trajectory PRIVATE ${OpenCV_LIBS})

add_library(show_replay STATIC
    options.cc
    log_parser.cc
    replay.cc
)
target_link_libraries(show_replay
    PUBLIC
    show_trajectory
    coverage_path_planning::common
    coverage_path_planning::viz
    coverage_path_planning::log
    coverage_path_planning::yaml_cpp
    coverage_path_planning::threads
)

add_executable(show main.cc)
target_link_libraries(show PRIVATE show_replay)
# Keep the existing build/tools/show executable path.
set_target_properties(show PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_executable(show_water_leak_test EXCLUDE_FROM_ALL
    ../../tests/show_water_leak_test.cc
)
target_link_libraries(show_water_leak_test PRIVATE
    show_replay foxglove_cpp
)
