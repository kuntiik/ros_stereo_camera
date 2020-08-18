cmake_minimum_required(VERSION 3.0.2)
project(bridge_test)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  pcl_conversions
)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

catkin_package(CATKIN_DEPENDS)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
set(pillar_corners_source_files
    src/pillar_corners.cpp
    src/pillar_corners.hpp)
add_library(pillar_corners ${pillar_corners_source_files})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(make_cloud src/make_cloud.cpp)
add_executable(save_cloud src/save_cloud.cpp)
add_executable(get_corners src/get_corners.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
target_link_libraries(make_cloud ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(get_corners ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES} pillar_corners)
target_link_libraries(save_cloud ${OpenCV_LIBS} ${catkin_LIBRARIES})
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

