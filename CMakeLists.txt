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
add_executable(${PROJECT_NAME}-haiku src/haiku.cpp)

# Playing around with Cartesian controller
add_executable(${PROJECT_NAME}-cartesian_control src/cartesian_control.cpp)
target_link_libraries(${PROJECT_NAME}-cartesian_control ${YARP_LIBRARIES} ICUB::iKin)

icubcontrib_add_uninstall_target()					# Enables uninstall
