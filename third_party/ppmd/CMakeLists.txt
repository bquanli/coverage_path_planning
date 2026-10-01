enable_language(C)

add_library(ppmd STATIC
    Alloc.c
    Ppmd8.c
    Ppmd8Dec.c
)
target_include_directories(ppmd PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")