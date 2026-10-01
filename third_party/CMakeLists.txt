# third_party/CMakeLists.txt

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

if(COVERAGE_WITH_FOXGLOVE)
  add_subdirectory(foxglove)
endif()

add_subdirectory(ppmd)

if(COVERAGE_WITH_RERUN)
  add_subdirectory(rerun)
endif()

# 测试框架。存在即用，缺失时由 tests/ 自动跳过，不阻塞主构建。
if(COVERAGE_BUILD_TESTS)
  find_package(GTest CONFIG)
  if(GTest_FOUND)
    add_library(coverage_gtest INTERFACE)
    add_library(coverage_path_planning::gtest ALIAS coverage_gtest)
    target_link_libraries(coverage_gtest INTERFACE GTest::gtest GTest::gtest_main)
  endif()
endif()
