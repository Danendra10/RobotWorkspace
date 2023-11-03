cmake_minimum_required(VERSION 3.0.2)
project(helpers)

add_compile_options(-std=c++17)

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

link_libraries(stdc++fs)

add_library(logger_lib src/logger.cpp)
target_link_libraries(logger_lib ${catkin_LIBRARIES})

add_library(vision_helper_lib src/vision_helper.cpp)
target_link_libraries(vision_helper_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(global_lib src/global.cpp)
target_link_libraries(global_lib ${catkin_LIBRARIES})

add_library(helper_lib src/helper.cpp)
target_link_libraries(helper_lib ${catkin_LIBRARIES})

add_library(pid_lib src/pid.cpp)
target_link_libraries(pid_lib ${catkin_LIBRARIES})

add_library(simple_math_lib src/simple_math.cpp)
target_link_libraries(simple_math_lib ${catkin_LIBRARIES})

add_library(potential_field_lib src/potential_field.cpp)
target_link_libraries(potential_field_lib ${catkin_LIBRARIES})