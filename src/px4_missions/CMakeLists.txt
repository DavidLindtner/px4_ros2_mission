cmake_minimum_required(VERSION 3.5)
project(px4_missions)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)




# simpleMission
add_executable(simpleMission src/simpleMission.cpp src/simple/droneSimple.cpp src/base/droneBase.cpp)
ament_target_dependencies(simpleMission rclcpp px4_msgs)

target_include_directories(simpleMission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS simpleMission
  DESTINATION lib/${PROJECT_NAME})





# missionSupervisor
add_executable(missionSupervisor src/missionSupervisor.cpp)
ament_target_dependencies(missionSupervisor rclcpp geometry_msgs)

target_include_directories(missionSupervisor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS missionSupervisor
  DESTINATION lib/${PROJECT_NAME})





# supervisedMission
add_executable(supervisedMission src/supervisedMission.cpp src/supervised/droneSupervised.cpp src/base/droneBase.cpp)
ament_target_dependencies(supervisedMission rclcpp px4_msgs geometry_msgs)

target_include_directories(supervisedMission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS supervisedMission
  DESTINATION lib/${PROJECT_NAME})





# all packages
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
