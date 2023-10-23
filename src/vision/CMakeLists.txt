cmake_minimum_required(VERSION 3.0.2)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  helpers
  cv_bridge
  image_transport
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL 1.7 REQUIRED)

if(PCL_FOUND)
  message(STATUS "PCL found")
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
else()
  message(STATUS "PCL not found")
endif()

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(capture src/capture.cpp)
target_link_libraries(capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} logger_lib)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} logger_lib vision_helper_lib global_lib)