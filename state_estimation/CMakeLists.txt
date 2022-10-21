cmake_minimum_required(VERSION 3.8)
project(state_estimation)

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

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

set(G2O_ROOT "/home/parallels/3rdparty/g2o")
file(GLOB G2O_LIBS ${G2O_ROOT}/lib/*.so)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_ROOT}/include
)

FILE(GLOB CPPS ${PROJECT_SOURCE_DIR}/src/*)
add_library(${PROJECT_NAME} SHARED
        ${CPPS}
        )

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${G2O_LIBS}
        )

ament_target_dependencies(${PROJECT_NAME} ${ros2_deps})

add_executable(state_estimation_node state_estimation_node.cpp)
target_link_libraries(state_estimation_node ${PROJECT_NAME})
ament_target_dependencies(state_estimation_node ${ros2_deps})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  state_estimation_node 
  DESTINATION lib/${PROJECT_NAME})


ament_package()
