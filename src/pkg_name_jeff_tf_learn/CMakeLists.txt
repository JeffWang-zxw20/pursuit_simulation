cmake_minimum_required(VERSION 3.0.2)
project(pkg_name_jeff_tf_learn)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
)


find_package(yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pkg_name_jeff_tf_learn
  CATKIN_DEPENDS roscpp rospy tf turtlesim
  DEPENDS system_lib
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)


add_executable(any_name_ok_launch_name src/file_tf_broadcaster.cpp)
target_link_libraries(any_name_ok_launch_name ${catkin_LIBRARIES})


add_executable(jeff_tf_listener src/file_tf_listener.cpp)
target_link_libraries(jeff_tf_listener ${catkin_LIBRARIES})

add_executable(create_new_frame src/file_create_frame.cpp)
target_link_libraries(create_new_frame ${catkin_LIBRARIES})


add_executable(cmake_list_name_move src/simulation_node_pid1.cpp
                                    src/car_model.cpp
                                    src/make_it_move.cpp
                                    src/map.cpp
                                    src/my_spline.cpp)
target_link_libraries(cmake_list_name_move ${catkin_LIBRARIES})
target_link_libraries(cmake_list_name_move yaml-cpp)
