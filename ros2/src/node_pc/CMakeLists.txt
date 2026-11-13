cmake_minimum_required(VERSION 3.16)
project(node_pc)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search kdtree)
foreach(pair
    "lidar_timestamp_shift_node|lidar_timestamp_shift"
    "pointcloud_merge_node|pointcloud_merge"
    "pointcloud_voxel_filter_node|pointcloud_voxel_filter"
    "pointcloud_colorize_node|pointcloud_colorize")
  string(REPLACE "|" ";" names "${pair}")
  list(GET names 0 target)
  list(GET names 1 source)
  add_executable(${target} src/${source}.cpp)
  target_include_directories(${target} PRIVATE include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(${target} rclcpp sensor_msgs std_msgs cv_bridge)
  target_link_libraries(${target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
  target_compile_options(${target} PRIVATE -O3)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(PROGRAMS scripts/image_enhancer.py scripts/monitor.py scripts/healthcheck.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/enclosure_correction.py scripts/runtime.py scripts/model_cpu.pt
  scripts/model_cpu.onnx scripts/stream_sample.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
