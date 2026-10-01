cmake_minimum_required(VERSION 3.28)
project(CoveragePathPlanning VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 小项目 / 整个项目统一 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(COVERAGE_BUILD_TESTS "构建单元测试目标" ON)
option(COVERAGE_ENABLE_SANITIZERS "为本项目的目标启用 ASan + UBSan" OFF)

# 告警与 sanitizer 通过两个接口目标传播，而不是用全局 CMAKE_CXX_FLAGS，
# 这样第三方代码不会被注入我们的告警，告警列表才有可能保持在零。
#   warnings   —— 仅影响目标自身的编译，用 PRIVATE 链接。
#   sanitizers —— 编译和链接均需生效且必须传递给可执行文件，用 PUBLIC 链接。
add_library(coverage_warnings INTERFACE)
add_library(coverage_path_planning::warnings ALIAS coverage_warnings)
target_compile_options(coverage_warnings INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wextra>
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wpedantic>
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wshadow>
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wnon-virtual-dtor>
)

add_library(coverage_sanitizers INTERFACE)
add_library(coverage_path_planning::sanitizers ALIAS coverage_sanitizers)
if(COVERAGE_ENABLE_SANITIZERS)
  target_compile_options(coverage_sanitizers INTERFACE
      -fsanitize=address,undefined -fno-omit-frame-pointer -g)
  target_link_options(coverage_sanitizers INTERFACE
      -fsanitize=address,undefined)
endif()

enable_testing()

add_subdirectory(third_party)
add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(tools)
add_subdirectory(examples)
