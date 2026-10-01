# clang-tidy 的接入方式：逐目标设置 CXX_CLANG_TIDY 属性，而不是设置全局的
# CMAKE_CXX_CLANG_TIDY。理由和告警接口目标一致——third_party/ 下是别人的代码，
# 我们既不想也无权去清理它的诊断；一旦把检查注入第三方目标，输出里永远有噪声，
# 这份检查最终就会被所有人忽略。
#
# 对外只暴露一个函数：
#   coverage_enable_clang_tidy(<target> [<target>...])
# 未启用时它是空操作，所以调用点不需要再写 if。

option(COVERAGE_CLANG_TIDY_WARNINGS_AS_ERRORS
    "把 clang-tidy 的诊断升级为编译错误" OFF)

# 命令存在 cache 里而不是普通变量：函数体在调用处的作用域里查找变量，
# 而 cache 变量在任何作用域都可见，这样就不依赖 include 的位置与顺序。
# CACHE INTERNAL 每次配置都会覆盖，因此把选项关掉后不会残留旧值。
set(COVERAGE_CLANG_TIDY_COMMAND "" CACHE INTERNAL "clang-tidy 的完整调用命令")

if(COVERAGE_ENABLE_CLANG_TIDY)
  # 允许用 -DCOVERAGE_CLANG_TIDY_EXECUTABLE=<路径> 指定；否则按名字探测。
  # 带版本号后缀是 Debian/Ubuntu 的打包习惯，必须一并探测。
  find_program(COVERAGE_CLANG_TIDY_EXECUTABLE
      NAMES clang-tidy
            clang-tidy-23 clang-tidy-21 clang-tidy-20 clang-tidy-19
            clang-tidy-18 clang-tidy-17 clang-tidy-16 clang-tidy-15
      DOC "clang-tidy 可执行文件")

  if(NOT COVERAGE_CLANG_TIDY_EXECUTABLE)
    message(FATAL_ERROR
        "COVERAGE_ENABLE_CLANG_TIDY=ON，但找不到 clang-tidy。"
        "请安装（Ubuntu: apt install clang-tidy），"
        "或用 -DCOVERAGE_CLANG_TIDY_EXECUTABLE=<路径> 指定。")
  endif()

  # --quiet 只抑制“检查了 N 个文件”这类统计行，诊断本身照常输出。
  # 编译器是 GCC 时，命令行里可能出现 clang 不认识的告警选项，
  # -Wno-unknown-warning-option 避免这些选项本身变成噪声。
  set(_clang_tidy_command
      "${COVERAGE_CLANG_TIDY_EXECUTABLE}"
      "--quiet"
      "--extra-arg=-Wno-unknown-warning-option")

  if(COVERAGE_CLANG_TIDY_WARNINGS_AS_ERRORS)
    list(APPEND _clang_tidy_command "--warnings-as-errors=*")
  endif()

  set(COVERAGE_CLANG_TIDY_COMMAND "${_clang_tidy_command}"
      CACHE INTERNAL "clang-tidy 的完整调用命令")

  message(STATUS "clang-tidy: ${COVERAGE_CLANG_TIDY_EXECUTABLE} "
                 "(warnings-as-errors=${COVERAGE_CLANG_TIDY_WARNINGS_AS_ERRORS})")
endif()

function(coverage_enable_clang_tidy)
  if(NOT COVERAGE_CLANG_TIDY_COMMAND)
    return()
  endif()

  foreach(target IN LISTS ARGN)
    if(NOT TARGET "${target}")
      message(FATAL_ERROR "coverage_enable_clang_tidy: 目标 ${target} 不存在")
    endif()
    set_property(TARGET "${target}" PROPERTY
        CXX_CLANG_TIDY "${COVERAGE_CLANG_TIDY_COMMAND}")
  endforeach()
endfunction()
