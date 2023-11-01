cmake_minimum_required(VERSION 3.8)
project(lidar_camera_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python  REQUIRED)

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include
${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Add Cpp executables
add_executable(lidar_camera_fusion_node src/lidar_camera_merge.cpp)
ament_target_dependencies(lidar_camera_fusion_node 
                          sensor_msgs message_filters rclcpp Eigen3 
                          PCL pcl_conversions OpenCV cv_bridge) 

target_link_libraries(lidar_camera_fusion_node
${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 

ament_python_install_package(${PROJECT_NAME})
# Install cpp executable
install(TARGETS lidar_camera_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  lidar_camera_fusion/image_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
