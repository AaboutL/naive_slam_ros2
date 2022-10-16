cmake_minimum_required(VERSION 3.8)
project(feature_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(ros2_deps 
rclcpp
std_msgs
sensor_msgs
geometry_msgs
cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

FILE(GLOB CPPS ${PROJECT_SOURCE_DIR}/src/*)
add_library(feature_tracking SHARED
        ${CPPS}
        )

target_link_libraries(feature_tracking
        ${OpenCV_LIBS}
        ${G2O_LIBS}
        )

ament_target_dependencies(feature_tracking  ${ros2_deps})

add_executable(feature_tracking_node feature_tracking_node.cpp)
target_link_libraries(feature_tracking_node feature_tracking)
ament_target_dependencies(feature_tracking_node ${ros2_deps})

install(TARGETS
  feature_tracking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  feature_tracking_node 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
