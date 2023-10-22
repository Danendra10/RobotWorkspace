cmake_minimum_required(VERSION 3.0.2)
project(helpers)

add_compile_options(-std=c++11)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(logger_lib src/logger.cpp)
target_link_libraries(logger_lib ${catkin_LIBRARIES})