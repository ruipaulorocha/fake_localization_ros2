cmake_minimum_required(VERSION 3.5)
project(fake_localization_ros2)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp_components REQUIRED)
#find_package(Boost REQUIRED)

# find_package(catkin REQUIRED
#         COMPONENTS
#             angles
#             geometry_msgs
#             message_filters
#             nav_msgs
#             rosconsole
#             roscpp
#             rospy
#             tf2_geometry_msgs
#             tf2_ros
#         )
#
# find_package(Boost REQUIRED)
#
# catkin_package(
#     CATKIN_DEPENDS
#         geometry_msgs
#         nav_msgs
#         roscpp
#         rospy
# )


#include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Install include directory for Cpp nodes
include_directories(include)

install(
 DIRECTORY include/
 DESTINATION include
)


# # Install and declare ROS component

# add_library(fake_localization_component SHARED src/FakeOdomNode.cpp)
# ament_target_dependencies(fake_localization_component
#   rclcpp rclcpp_components
#   nav_msgs geometry_msgs tf2_ros nav_msgs tf2_geometry_msgs tf2 angles
# )
# rclcpp_components_register_node(
#     fake_localization_component
#     PLUGIN "fake_localization_ros2::FakeOdomNode"
#     EXECUTABLE exec_fake_localization_component
# )
# ament_export_targets(export_fake_localization_component)
# install(
#     TARGETS  fake_localization_component
#     EXPORT   export_fake_localization_component
#     ARCHIVE  DESTINATION lib    
#     LIBRARY  DESTINATION lib
#     RUNTIME  DESTINATION bin
#     INCLUDES DESTINATION include
# )


# Declare and install Cpp node

add_executable(fake_localization src/fake_localization.cpp)
ament_target_dependencies(fake_localization
  rclcpp nav_msgs geometry_msgs tf2_ros nav_msgs tf2_geometry_msgs tf2 angles)
#target_link_libraries(fake_localization
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#    )
#add_dependencies(fake_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(
    TARGETS
        fake_localization
    DESTINATION lib/${PROJECT_NAME}
)

# install(
#     TARGETS
#         fake_localization
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
