cmake_minimum_required(VERSION 3.8)
project(positioning_ben_kalman)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gps_msgs REQUIRED)

include_directories(
  include
 ${EIGEN3_INCLUDE_DIRS}
 )

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

add_executable(positioning_ben src/Positioning_Ben_Kalman.cpp src/tf_broadcaster.cpp src/UTM.cpp src/kalman.cpp)
ament_target_dependencies(positioning_ben rclcpp std_msgs geometry_msgs std_srvs sensor_msgs nav_msgs tf2 tf2_ros Eigen3 gps_msgs)

install(TARGETS
    positioning_ben
    DESTINATION lib/${PROJECT_NAME}
)
