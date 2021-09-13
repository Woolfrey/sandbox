cmake_minimum_required(VERSION 2.8.9)					# Minimum cmake requirement

project(sandbox)							# ${PROJECT_NAME}

find_package(YARP 3.3.0 REQUIRED)

# A simple poem in haiku format
add_executable(${PROJECT_NAME}-haiku src/haiku.cpp)


# Joint controller
add_executable(${PROJECT_NAME}-position_control src/position_control.cpp)
target_link_libraries(${PROJECT_NAME}-position_control ${YARP_LIBRARIES})
