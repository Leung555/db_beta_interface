cmake_minimum_required(VERSION 3.8)
project(db_beta_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(dynamixel_sdk REQUIRED)
# find_package(dynamixel_workbench REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# set dependencies
set(dependencies
rclcpp
rclpy
)

# Build
include_directories(
  include
)

add_executable(position_controller src/position_controller.cpp)
ament_target_dependencies(position_controller
  dynamixel_workbench_toolbox
  dynamixel_workbench_msgs
  dynamixel_sdk
  rclcpp
  std_msgs
  sensor_msgs
)
# add_executable(position_controller_XL320 src/position_controller_XL320.cpp)
# ament_target_dependencies(position_controller_XL320
#   dynamixel_workbench_toolbox
#   dynamixel_workbench_msgs
#   dynamixel_sdk
#   rclcpp
#   std_msgs
#   sensor_msgs
# )
add_executable(reboot src/reboot.cpp)
ament_target_dependencies(reboot
  dynamixel_workbench_toolbox
  dynamixel_workbench_msgs
  dynamixel_sdk
  rclcpp
  std_msgs
  sensor_msgs
)


# Install
install(TARGETS
  position_controller
  reboot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/cpg.py
  DESTINATION lib/${PROJECT_NAME}
)

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

#PY --------------------------------------------------------------------
# #install python modules
# ament_python_install_package(${PROJECT_NAME})

# #install python executables
# install(PROGRAMS
#   scripts/cpg.py
#   DESTINATION lib/${PROJECT_NAME})

#--------------------------------------------------------------------
# export dependencies
ament_export_dependencies(${dependencies})
ament_package()
