cmake_minimum_required(VERSION 2.8.9)					# Minimum cmake requirement

project(sandbox)							# ${PROJECT_NAME}


find_package(Eigen3 REQUIRED)
#find_package(ICUB REQUIRED)						# Links below to ICUB::iKin
find_package(iDynTree REQUIRED)						# Links below to iDynTree
find_package(YARP 3.3.0 REQUIRED)					# Links below to ${YARP_LIBRARIES}

find_package(RobotLibrary REQUIRED)


## I think this enables installation of gazebo modules in iCub direction
find_package(ICUBcontrib REQUIRED)					# Mandatory
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})		# Extend the current search path to load helpers
include(ICUBcontribHelpers)						# I don't know what this does
include(ICUBcontribOptions)						# I don't know what this does
#icubcontrib_set_default_prefix()

include_directories(include)						# Location of header files

add_subdirectory(gazebo)						# Location of other CMakeLists

# A simple poem in haiku format
add_executable(haiku src/haiku.cpp)

# Cartesian impedance control
add_executable(impedance_control src/impedance_control.cpp)
target_link_libraries(impedance_control Eigen3::Eigen iDynTree::idyntree-high-level ${YARP_LIBRARIES})

add_executable(math_test src/math_test.cpp)
target_link_libraries(math_test Eigen3::Eigen)

icubcontrib_add_uninstall_target()					# Enables uninstall
