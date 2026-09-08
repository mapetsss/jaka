cmake_minimum_required(VERSION 3.8)
project(jaka_s5_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(jaka_s5_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(s5_moveit_server src/s5_moveit_server.cpp)
ament_target_dependencies(s5_moveit_server
  control_msgs
  jaka_s5_interfaces
  rclcpp
  rclcpp_action
  sensor_msgs
  std_srvs
)
target_include_directories(s5_moveit_server PUBLIC include)

add_executable(s5_joint_controller src/s5_joint_controller.cpp)
ament_target_dependencies(s5_joint_controller
  jaka_s5_interfaces
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  std_srvs
)
target_include_directories(s5_joint_controller PUBLIC include)
target_link_libraries(s5_joint_controller Qt5::Widgets)

add_executable(s5_ready_pose src/s5_ready_pose.cpp)
ament_target_dependencies(s5_ready_pose
  moveit_core
  moveit_ros_planning_interface
  rclcpp
)
target_include_directories(s5_ready_pose PUBLIC include)

set(JAKA_API_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/libjakaAPI.so)
target_link_libraries(s5_moveit_server ${JAKA_API_LIBRARY})
set_target_properties(s5_moveit_server PROPERTIES INSTALL_RPATH "$ORIGIN")

install(TARGETS
  s5_moveit_server
  s5_joint_controller
  s5_ready_pose
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${JAKA_API_LIBRARY} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_s5_joint_utils test/test_s5_joint_utils.cpp)
  if(TARGET test_s5_joint_utils)
    target_include_directories(test_s5_joint_utils PUBLIC include)
  endif()
endif()

ament_package()
