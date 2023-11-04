cmake_minimum_required(VERSION 3.0.2)
project(master)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  helpers
  motion
  multirole
  role_attacker
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES master
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(master src/master.cpp)
target_link_libraries(master ${catkin_LIBRARIES} logger_lib motion_lib simple_math_lib multirole_lib role_attacker_lib)