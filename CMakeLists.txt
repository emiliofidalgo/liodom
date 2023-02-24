cmake_minimum_required(VERSION 3.5)
project(liodom LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(std_msgs)
find_package(geometry_msgs)
find_package(nav_msgs)
find_package(sensor_msgs)  
find_package(tf2)
find_package(tf2_eigen)
find_package(tf2_ros)
find_package(pcl_conversions)
find_package(pcl_ros)

find_package(PCL REQUIRED)
find_package(Eigen3)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###########
## Build ##
###########

include_directories(include
                    ${PCL_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

### Targets ###
# Set library sources
set(${PROJECT_NAME}_sources
      src/shared_data.cc
      src/stats.cc
      src/params.cc
      src/feature_extractor.cc
      src/laser_odometry.cc)

#### Laser Odometry
add_executable(liodom_node
      src/liodom_node.cc
      ${${PROJECT_NAME}_sources})
target_link_libraries(liodom_node ${PCL_LIBRARIES} ${CERES_LIBRARIES})
ament_target_dependencies(liodom_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_eigen tf2_ros pcl_conversions pcl_ros)

# #### Laser Mapping
add_executable(liodom_mapping
               src/map.cc
               src/liodom_mapping_node.cc)
target_link_libraries(liodom_mapping ${PCL_LIBRARIES})
ament_target_dependencies(liodom_mapping rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_eigen tf2_ros pcl_conversions pcl_ros)

install(TARGETS
  liodom_node
  liodom_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  rviz 
  DESTINATION share/${PROJECT_NAME})

ament_package()