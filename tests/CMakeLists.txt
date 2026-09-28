add_executable(stc_debug stc_debug.cc)
target_link_libraries(stc_debug
    PRIVATE
    foxglove_cpp
)