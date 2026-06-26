cmake_minimum_required(VERSION 3.22.1)
project(flexiv_rt_torque_collector)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

find_package(flexiv_rdk REQUIRED)

add_executable(flexiv_rt_torque_sysid flexiv_rt_torque_sysid.cpp)
target_link_libraries(flexiv_rt_torque_sysid flexiv::flexiv_rdk)
set_target_properties(flexiv_rt_torque_sysid PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON)


add_executable(flexiv_zero_torque_drift flexiv_zero_torque_drift.cpp)
target_link_libraries(flexiv_zero_torque_drift flexiv::flexiv_rdk)
