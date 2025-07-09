cmake_minimum_required(VERSION 3.0.2)
project(ackermann_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf nav_msgs geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 可执行文件
add_executable(mppi_controller src/mppi_controller.cpp)
add_executable(path_planner src/path_planner.cpp)
add_executable(costmap_manager src/costmap_manager.cpp)
add_executable(fake_odom_publisher src/fake_odom_publisher.cpp)

# 链接库
target_link_libraries(mppi_controller ${catkin_LIBRARIES})
target_link_libraries(path_planner ${catkin_LIBRARIES})
target_link_libraries(costmap_manager ${catkin_LIBRARIES})
target_link_libraries(fake_odom_publisher ${catkin_LIBRARIES})

# 安装文件
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(TARGETS mppi_controller path_planner costmap_manager fake_odom_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
