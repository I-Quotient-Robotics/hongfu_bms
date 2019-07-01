cmake_minimum_required(VERSION 2.8.3)
project(hongfu_bms_status)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
  hongfu_bms_msg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hongfu_bms
#  CATKIN_DEPENDS roscpp serial std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/hongfu_bms_status_node.cpp src/hongfu_bms_status.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
