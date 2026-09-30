# thrid_party/CMakeLists.txt

find_package(yaml-cpp CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

add_library(coverage_yaml_cpp INTERFACE)
add_library(coverage_path_planning::yaml_cpp ALIAS coverage_yaml_cpp)
target_link_libraries(coverage_yaml_cpp INTERFACE yaml-cpp::yaml-cpp)

add_library(coverage_spdlog INTERFACE)
add_library(coverage_path_planning::spdlog ALIAS coverage_spdlog)
target_link_libraries(coverage_spdlog INTERFACE spdlog::spdlog)

add_library(coverage_eigen3 INTERFACE)
add_library(coverage_path_planning::eigen3 ALIAS coverage_eigen3)
target_link_libraries(coverage_eigen3 INTERFACE Eigen3::Eigen)

add_library(coverage_threads INTERFACE)
add_library(coverage_path_planning::threads ALIAS coverage_threads)
target_link_libraries(coverage_threads INTERFACE Threads::Threads)

add_subdirectory(foxglove)
add_subdirectory(ppmd)
add_subdirectory(rerun)
