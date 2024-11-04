# Add an executable
add_executable(grasp main.cpp canAPI.cpp)

# Link with required libraries
target_link_libraries(grasp BHand pcanbasic pthread ${ZMQ_LIBRARIES})

# Install the executable
install(TARGETS grasp DESTINATION ${PROJECT_BINARY_DIR}/bin)
