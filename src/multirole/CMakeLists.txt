cmake_minimum_required(VERSION 3.0.2)
project(multirole)

find_package(catkin REQUIRED COMPONENTS )

catkin_package(
  INCLUDE_DIRS include  
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib 
  src/${PROJECT_NAME}.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})