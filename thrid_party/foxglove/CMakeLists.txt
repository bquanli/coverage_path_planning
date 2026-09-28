# Respect foxglove-sdk_DIR and CMAKE_PREFIX_PATH before the local fallback.
find_package(
    foxglove-sdk
    CONFIG
    REQUIRED
    PATHS /opt/sdk/foxglove
)

foxglove_sdk_add_cpp_library(
    foxglove_cpp
    TYPE SHARED
)
