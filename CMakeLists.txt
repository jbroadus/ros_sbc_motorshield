cmake_minimum_required(VERSION 2.8.3)
project(ros_sbc_motorshield)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_motorshield src/motorshield.cpp src/dc_motor.cpp)

add_dependencies(${PROJECT_NAME}_motorshield ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_motorshield
  ${catkin_LIBRARIES}
  pigpio
)

install(TARGETS ${PROJECT_NAME}_motorshield
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
