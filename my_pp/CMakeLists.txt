cmake_minimum_required(VERSION 3.8)
project(my_pp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

set(dependencies 
  rclcpp
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools

)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(pick_place src/pick_place.cpp)
target_include_directories(pick_place PUBLIC include)
ament_target_dependencies( pick_place ${dependencies} )

add_executable(deneme src/deneme.cpp)
target_include_directories(deneme PUBLIC include)
ament_target_dependencies( deneme ${dependencies} )

add_executable(gripper src/gripper.cpp)
target_include_directories(gripper PUBLIC include)
ament_target_dependencies( gripper ${dependencies} )

add_executable(arm src/arm.cpp)
target_include_directories(arm PUBLIC include)
ament_target_dependencies( arm ${dependencies} )

install(TARGETS pick_place deneme gripper arm
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
