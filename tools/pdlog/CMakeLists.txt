add_library(pdlog_core STATIC
    application.cc
    config.cc
    log_reader.cc
    ppmd_decoder.cc
)
target_include_directories(pdlog_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(pdlog_core PRIVATE
    ppmd coverage_path_planning::yaml_cpp
)
add_executable(pdlog pdlog.cc)
target_link_libraries(pdlog PRIVATE pdlog_core)
target_compile_definitions(pdlog PRIVATE
    COVERAGE_PATH_PLANNING_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

add_executable(pdlog_test EXCLUDE_FROM_ALL ../../tests/pdlog_test.cc)
target_link_libraries(pdlog_test PRIVATE pdlog_core)
