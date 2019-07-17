cmake_minimum_required(VERSION 2.8.3)
project(unity_world)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  apriltags2_ros
  visualization_msgs
  tf
  moveit_ros_planning_interface
  pr2_phantom
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib src/limited_marker_queue.cpp include/${PROJECT_NAME}/limited_marker_queue.h)
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})


add_executable(unity_world src/unity_world.cpp)
target_link_libraries(unity_world ${catkin_LIBRARIES})
target_link_libraries(unity_world ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
add_dependencies(unity_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(marker_to_collision_object src/marker_to_collision_object.cpp)
target_link_libraries(marker_to_collision_object ${catkin_LIBRARIES})
target_link_libraries(marker_to_collision_object ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
add_dependencies(marker_to_collision_object ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
