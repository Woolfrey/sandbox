cmake_minimum_required(VERSION 2.8.9)					# Minimum cmake requirement

project(sandbox)							# ${PROJECT_NAME}

find_package(YARP 3.3.0 REQUIRED)					# Links below to ${YARP_LIBRARIES}

find_package(ICUB REQUIRED)						# Links below to ICUB::iKin

## I think this enables installation of gazebo modules in iCub direction
find_package(ICUBcontrib REQUIRED)					# Mandatory
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})		# Extend the current search path to load helpers
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

include_directories(include)						# Location of header files

add_subdirectory(gazebo)						# Location of other CMakeLists

# A simple poem in haiku format
add_executable(haiku src/haiku.cpp)

# Simple position control in the joint space
add_executable(position_control src/position_control.cpp)
target_link_libraries(position_control ${YARP_LIBRARIES})

# Custom joint space velocity control
add_executable(velocity_control src/velocity_control.cpp)
target_link_libraries(velocity_control ${YARP_LIBRARIES})

# Playing around with Cartesian controller
add_executable(cartesian_control src/cartesian_control.cpp)
target_link_libraries(cartesian_control ${YARP_LIBRARIES} ICUB::iKin)

# Testing the threader class
add_executable(threading src/threading.cpp)
target_link_libraries(threading ${YARP_LIBRARIES})

# Demonstrates stiffness optimization
add_executable(stiffness_control src/stiffness_control.cpp)
target_link_libraries(stiffness_control ${YARP_LIBRARIES} ICUB::iKin)

icubcontrib_add_uninstall_target()					# Enables uninstall
