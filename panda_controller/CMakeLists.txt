cmake_minimum_required(VERSION 3.8)
project(panda_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(panda_world REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# below sets up code coverage and uncrustify config file location
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
  set(ARG_CONFIG_FILE "${PROJECT_SOURCE_DIR}/uncrustify_google_cpp_allman.cfg"
    CACHE STRING "uncrustify config file")
endif()

# # build my  library
# add_library(panda_controller_lib
#   src/motion_stack.cpp
#   src/img_stack.cpp)
# target_include_directories(panda_controller_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

include_directories(include/panda_controller)

add_executable(motion src/motion_node.cpp src/motion_stack.cpp src/motion_stack_node.cpp)
ament_target_dependencies(motion rclcpp std_msgs geometry_msgs tf2 tf2_ros moveit_ros_planning_interface panda_world)

add_executable(testpub src/test.cpp)
ament_target_dependencies(testpub rclcpp geometry_msgs panda_world)

add_executable(img src/img_node.cpp src/img_stack.cpp)
ament_target_dependencies(img rclcpp std_msgs geometry_msgs tf2 tf2_ros OpenCV cv_bridge sensor_msgs panda_world image_transport)

install(TARGETS
  motion
  img
  testpub
  DESTINATION lib/${PROJECT_NAME})

# # link the executables with my dummy library
# target_link_directories(motion PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
# target_link_libraries(motion panda_controller_lib)
# target_link_directories(img PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
# target_link_libraries(img panda_controller_lib)

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



  # Add a gtest executable
  set(TESTFILES
    test/main.cpp
    test/test.cpp src/motion_stack.cpp src/img_stack.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test  rclcpp std_msgs geometry_msgs
    tf2 tf2_ros OpenCV cv_bridge sensor_msgs panda_world image_transport
    moveit_ros_planning_interface)
endif()

install(DIRECTORY src launch DESTINATION share/${PROJECT_NAME})

# Install coverage generator executable
install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})
ament_package()
