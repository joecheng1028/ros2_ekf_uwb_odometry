cmake_minimum_required(VERSION 3.8)
project(cpp_fundamentals)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(hello_node src/hello_node.cpp)
ament_target_dependencies(hello_node rclcpp)
target_include_directories(hello_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS hello_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(talker_node src/talker_node.cpp)
ament_target_dependencies(talker_node rclcpp std_msgs)
install(TARGETS talker_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(listener_node src/listener_node.cpp)
ament_target_dependencies(listener_node rclcpp std_msgs)
install(TARGETS listener_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(counter_node src/counter_node.cpp)
ament_target_dependencies(counter_node rclcpp std_msgs)
install(TARGETS counter_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp std_srvs)
install(TARGETS service_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp std_srvs)
install(TARGETS service_client
  DESTINATION lib/${PROJECT_NAME})

add_executable(param_node src/param_node.cpp)
ament_target_dependencies(param_node rclcpp)
install(TARGETS param_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UwbRange.msg"
  DEPENDENCIES builtin_interfaces
)

add_executable(tf2_broadcaster src/tf2_broadcaster.cpp)
ament_target_dependencies(tf2_broadcaster rclcpp geometry_msgs tf2_ros)
install(TARGETS tf2_broadcaster DESTINATION lib/${PROJECT_NAME})

add_executable(tf2_listener src/tf2_listener.cpp)
ament_target_dependencies(tf2_listener rclcpp geometry_msgs tf2_ros tf2)
install(TARGETS tf2_listener DESTINATION lib/${PROJECT_NAME})

add_executable(uwb_publisher src/uwb_publisher.cpp)
ament_target_dependencies(uwb_publisher rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(uwb_publisher ${cpp_typesupport_target})
install(TARGETS uwb_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(uwb_listener src/uwb_listener.cpp)
ament_target_dependencies(uwb_listener rclcpp geometry_msgs)
install(TARGETS uwb_listener DESTINATION lib/${PROJECT_NAME})

add_executable(realtime_node src/realtime_node.cpp)
ament_target_dependencies(realtime_node rclcpp geometry_msgs)
install(TARGETS realtime_node DESTINATION lib/${PROJECT_NAME})

add_executable(eigen_pdop src/eigen_pdop.cpp)
target_include_directories(eigen_pdop PUBLIC ${EIGEN3_INCLUDE_DIR})
install(TARGETS eigen_pdop DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
  
add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp geometry_msgs nav_msgs)

target_include_directories(ekf_node PUBLIC ${EIGEN3_INCLUDE_DIR})
install(TARGETS ekf_node DESTINATION lib/${PROJECT_NAME})


ament_package()
