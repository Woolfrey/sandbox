cmake_minimum_required(VERSION 2.8.9)					# Minimum cmake requirement

project(sandbox)							# ${PROJECT_NAME}

find_package(YARP 3.3.0 REQUIRED)
find_package(ICUB REQUIRED)

include_directories(
	include
)

# A simple poem in haiku format
add_executable(${PROJECT_NAME}-haiku src/haiku.cpp)

# Single arm position controller
#add_executable(${PROJECT_NAME}-position_control src/position_control.cpp)
#target_link_libraries(${PROJECT_NAME}-position_control ${YARP_LIBRARIES})

# Dual arm position controller
#add_executable(${PROJECT_NAME}-dual_position_control src/position_control2.cpp)
#target_link_libraries(${PROJECT_NAME}-dual_position_control ${YARP_LIBRARIES})

# Joint level velocity control
#add_executable(${PROJECT_NAME}-velocity_control src/velocity_control.cpp)
#target_link_libraries(${PROJECT_NAME}-velocity_control ${YARP_LIBRARIES})

# Custom joint velocity control
#add_executable(${PROJECT_NAME}-velocity_control2 src/velocity_control2.cpp)
#target_link_libraries(${PROJECT_NAME}-velocity_control2 ${YARP_LIBRARIES})

# Dual arm velocity control
#add_executable(${PROJECT_NAME}-dual_velocity_control src/velocity_control3.cpp)
#target_link_libraries(${PROJECT_NAME}-dual_velocity_control ${YARP_LIBRARIES})

# Threading example
#add_executable(${PROJECT_NAME}-threading src/threading.cpp)
#target_link_libraries(${PROJECT_NAME}-threading ${YARP_LIBRARIES})

# Playing around with Cartesian controller
add_executable(${PROJECT_NAME}-cartesian_control src/cartesian_control.cpp)
target_link_libraries(${PROJECT_NAME}-cartesian_control ${YARP_LIBRARIES} ICUB::iKin)
