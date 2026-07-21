cmake_minimum_required(VERSION 3.8)
project(vio_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(TBB REQUIRED)

add_library(vio_gps_control SHARED src/vio_gps_control.cpp)
target_include_directories(vio_gps_control PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(vio_gps_control ${GAZEBO_LIBRARIES})
target_compile_options(vio_gps_control PRIVATE ${GAZEBO_CXX_FLAGS})
set_target_properties(vio_gps_control PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

install(TARGETS vio_gps_control LIBRARY DESTINATION lib)

add_executable(vio_gps_control_command src/vio_gps_control_command.cpp)
target_include_directories(vio_gps_control_command PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(vio_gps_control_command ${GAZEBO_LIBRARIES} TBB::tbb)
target_compile_options(vio_gps_control_command PRIVATE ${GAZEBO_CXX_FLAGS})
set_target_properties(vio_gps_control_command PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
install(TARGETS vio_gps_control_command RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("env-hooks/vio_gazebo_plugins.dsv.in")
ament_package()
