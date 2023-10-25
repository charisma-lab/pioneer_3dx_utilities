cmake_minimum_required(VERSION 3.0.2)
project(pioneer_3dx_utilities)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/ps4_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


#add_executable(pioneer3dx_tf_broadcaster src/pioneer3dx_tf_broadcaster.cpp)
#target_link_libraries(pioneer3dx_tf_broadcaster ${catkin_LIBRARIES})
#add_dependencies(pioneer3dx_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(pioneer3dx_odometry_publisher src/pioneer3dx_odometry_publisher.cpp)
target_link_libraries(pioneer3dx_odometry_publisher ${catkin_LIBRARIES})
add_dependencies(pioneer3dx_odometry_publisher ${catkin_EXPORTED_TARGETS})

#add_executable(odom_map_frame_publisher src/odom_map_frame_publisher.cpp)
#target_link_libraries(odom_map_frame_publisher ${catkin_LIBRARIES})
#add_dependencies(odom_map_frame_publisher ${catkin_EXPORTED_TARGETS})

#add_executable(laser_tf_broadcaster src/laser_tf_broadcaster.cpp)
#target_link_libraries(laser_tf_broadcaster ${catkin_LIBRARIES})
#add_dependencies(laser_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(rp_laser_tf_broadcaster src/rp_laser_tf_broadcaster.cpp)
target_link_libraries(rp_laser_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(rp_laser_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(rplidar_laser_publisher src/rplidar_laser_publisher.cpp)
target_link_libraries(rplidar_laser_publisher ${catkin_LIBRARIES})
add_dependencies(rplidar_laser_publisher ${catkin_EXPORTED_TARGETS})

#add_executable(pioneer3dx_pid_controller src/pioneer3dx_pid_controller.cpp)
#target_link_libraries(pioneer3dx_pid_controller ${catkin_LIBRARIES})
#add_dependencies(pioneer3dx_pid_controller ${catkin_EXPORTED_TARGETS})

#add_executable(pioneer3dx_move_base_recover src/move_base_recover.cpp)
#target_link_libraries(pioneer3dx_move_base_recover ${catkin_LIBRARIES})
#add_dependencies(pioneer3dx_move_base_recover ${catkin_EXPORTED_TARGETS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
