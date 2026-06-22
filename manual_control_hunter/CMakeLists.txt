cmake_minimum_required(VERSION 3.8)
project(manual_control_hunter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(SDL2 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

if(BUILD_TESTING)
set(BUILD_TESTING false) 
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

ament_package()

add_executable(keyboard src/Keyboard_Control.cpp)
add_executable(joystick src/Joystick_Control.cpp)
ament_target_dependencies(keyboard rclcpp std_msgs geometry_msgs)
ament_target_dependencies(joystick rclcpp std_msgs geometry_msgs SDL2 std_srvs sensor_msgs)

install(TARGETS
    keyboard
    joystick
    DESTINATION lib/${PROJECT_NAME}
)
