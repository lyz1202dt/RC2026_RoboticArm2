cmake_minimum_required(VERSION 3.8)
project(parameter_measure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_interfaces REQUIRED)

set(PINOCCHIO_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/pinocchio")
if(EXISTS "${PINOCCHIO_VENDOR_DIR}/CMakeLists.txt")
  set(PINOCCHIO_WITH_HPP_FCL OFF CACHE BOOL "Build Pinocchio without hpp-fcl support" FORCE)
  set(PINOCCHIO_WITH_CASADI OFF CACHE BOOL "Build Pinocchio without CasADi support" FORCE)
  set(PINOCCHIO_WITH_CPPAD OFF CACHE BOOL "Build Pinocchio without CppAD support" FORCE)
  set(PINOCCHIO_WITH_OPENMP OFF CACHE BOOL "Build Pinocchio without OpenMP support" FORCE)
  add_subdirectory("${PINOCCHIO_VENDOR_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/pinocchio")
else()
  find_package(pinocchio REQUIRED)
endif()

add_executable(parameter_measure_node
  src/main.cpp
  src/parameter_measure_node.cpp
  src/record.cpp
  src/trajectory.cpp
)

target_include_directories(parameter_measure_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(parameter_measure_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  ament_index_cpp
  robot_interfaces
)

target_link_libraries(parameter_measure_node
  Eigen3::Eigen
  pinocchio::pinocchio
)

install(TARGETS
  parameter_measure_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
