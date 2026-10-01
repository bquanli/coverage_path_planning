cmake_minimum_required(VERSION 3.28)
project(CoveragePathPlanning VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 小项目 / 整个项目统一 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(COVERAGE_BUILD_TESTS "构建单元测试目标" ON)
option(COVERAGE_ENABLE_SANITIZERS "为本项目的目标启用 ASan + UBSan" OFF)
option(COVERAGE_ENABLE_CLANG_TIDY "构建时对本项目目标运行 clang-tidy" OFF)

# 可视化 SDK 是可选依赖：它们需要手工安装到 /opt/sdk，干净机器（例如 CI）上并不
# 存在。做成可选之后，没有 SDK 的环境依然能配置、构建并测试与可视化无关的核心
# 部分；一个跑不起来的 CI 等于没有 CI。
option(COVERAGE_WITH_FOXGLOVE "启用 foxglove 可视化后端（需要 foxglove SDK）" ON)
option(COVERAGE_WITH_RERUN "启用 rerun 可视化后端（需要 rerun SDK）" ON)

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

# clang-tidy 的探测与 coverage_enable_clang_tidy() 函数。
# 必须在 add_subdirectory 之前 include，子目录里才能调用到这个函数。
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(ClangTidy)

enable_testing()

add_subdirectory(third_party)
add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(tools)
add_subdirectory(examples)
