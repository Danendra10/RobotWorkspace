cmake_minimum_required(VERSION 3.0.2)
project(motion)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS 
helpers
)
find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(motion_lib 
  src/motion.cpp
)
target_link_libraries(motion_lib ${catkin_LIBRARIES} pid_lib logger_lib global_lib potential_field_lib)