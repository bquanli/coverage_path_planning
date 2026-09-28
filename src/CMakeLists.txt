add_library(coverage_log INTERFACE)
add_library(coverage_path_planning::log ALIAS coverage_log)
target_include_directories(coverage_log INTERFACE "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(coverage_log INTERFACE coverage_path_planning::spdlog)

add_library(viz_lib SHARED
    viz/viz.cc
    viz/backend.cc
    viz/converters.cc
)
add_library(coverage_path_planning::viz ALIAS viz_lib)
add_library(viz_lib::viz_lib ALIAS viz_lib)
target_include_directories(viz_lib PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(viz_lib
    PUBLIC coverage_path_planning::eigen3
    PRIVATE foxglove_cpp coverage_path_planning::threads
)

add_library(coverage_common SHARED
  common/robot_model/robot_model_helper.cc
)
add_library(coverage_path_planning::common ALIAS coverage_common)
target_include_directories(coverage_common PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(coverage_common
    PUBLIC coverage_path_planning::eigen3 coverage_path_planning::yaml_cpp
    PRIVATE coverage_path_planning::log
)

add_executable(main main.cc)
target_link_libraries(main PRIVATE coverage_path_planning::viz coverage_path_planning::threads)
