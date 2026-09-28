find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

add_library(planning_viz SHARED
    viz.cc
    backend.cc
    converters.cc
)
add_library(planning_viz::planning_viz ALIAS planning_viz)
target_compile_features(planning_viz PUBLIC cxx_std_20)
target_include_directories(planning_viz PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(planning_viz
    PUBLIC Eigen3::Eigen
    PRIVATE foxglove_cpp Threads::Threads
)
set_target_properties(planning_viz PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_executable(main main.cc)
target_link_libraries(main PRIVATE planning_viz::planning_viz Threads::Threads)