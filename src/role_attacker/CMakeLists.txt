cmake_minimum_required(VERSION 3.0.2)
project(role_attacker)

find_package(catkin REQUIRED COMPONENTS
    multirole
    motion
    helpers
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} multirole_lib motion_lib helper_lib simple_math_lib)