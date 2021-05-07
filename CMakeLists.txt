cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(liodom)

# Setting build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Setting the flags for profiling information or not
if(CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "Setting Release options")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall -O3 -g")
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Setting Debug options")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -O1 -pg")
endif()

# Check C++ support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   message(STATUS "Using flag -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   message(STATUS "Using flag -std=c++11")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14/11/0x support.
                        Please use a different C++ compiler.")
endif()

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  eigen_conversions
  tf_conversions
  pcl_conversions
)

# Other packages
find_package(Eigen3)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Printing the compiling flags
message(STATUS "Compiler flags: ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf eigen_conversions tf_conversions pcl_conversions
  DEPENDS EIGEN3 PCL CERES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

#### Laser Odometry
add_library(liodom
            src/shared_data.cc
            src/stats.cc
            src/params.cc
            src/feature_extractor.cc
            src/laser_odometry.cc)
target_link_libraries(liodom
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${CERES_LIBRARIES}
                      )

add_executable(liodom_node
               src/liodom_node.cc)
target_link_libraries(liodom_node
                      liodom
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${CERES_LIBRARIES}
                      )

#### Laser Mapping
add_executable(liodom_mapping
               src/map.cc
               src/liodom_mapping_node.cc)
target_link_libraries(liodom_mapping
               ${PCL_LIBRARIES}
               ${catkin_LIBRARIES} 
               )