find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs opencv_highgui)
add_executable(gridmap_to_polygon gridmap_to_polygon.cc)
target_include_directories(gridmap_to_polygon PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gridmap_to_polygon PRIVATE opencv_core opencv_imgproc opencv_imgcodecs)

if(COVERAGE_WITH_FOXGLOVE)
  add_executable(gridmap_gen grid_map_gen.cc)
  target_include_directories(gridmap_gen PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(gridmap_gen PRIVATE opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui foxglove_cpp)
endif()




if(COVERAGE_WITH_RERUN)
  add_executable(rgridmap_gen grid_map_gen_rerun.cc)
  target_link_libraries(rgridmap_gen
      PRIVATE
      coverage_path_planning::rerun opencv_core opencv_imgproc opencv_imgcodecs
  )
endif()