find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

add_library(viz_lib SHARED
    viz/viz.cc
    viz/backend.cc
    viz/converters.cc
)
add_library(viz_lib::viz_lib ALIAS viz_lib)
target_compile_features(viz_lib PUBLIC cxx_std_20)
target_include_directories(viz_lib PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(viz_lib
    PUBLIC Eigen3::Eigen
    PRIVATE foxglove_cpp Threads::Threads
)
set_target_properties(viz_lib PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_executable(main main.cc)
target_link_libraries(main PRIVATE viz_lib::viz_lib Threads::Threads)