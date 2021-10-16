# These commands ensure that gazebo knows where to find the models

#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION share/gazebo)		# Install the models

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds DESTINATION share/gazebo)		# Install worlds
