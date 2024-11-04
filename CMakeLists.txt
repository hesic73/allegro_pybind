cmake_minimum_required(VERSION 3.15)
project(AllegroHand LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable pybind11's automatic Python detection
set(PYBIND11_FINDPYTHON ON)
set(pybind11_DIR /home/abrar/hsc/pybind11-2.6.2/build/lib/pybind11/share/cmake/pybind11)
find_package(pybind11 CONFIG REQUIRED)

# Define the source files
set(SOURCES src/main.cpp src/canAPI.cpp)

# Create the Python module with the desired name
pybind11_add_module(allegro_zmq_pybind ${SOURCES})

# Link required libraries
target_link_libraries(allegro_zmq_pybind PRIVATE BHand pcanbasic pthread ${ZMQ_LIBRARIES})

# Install the module to the current directory for Python import
install(TARGETS allegro_zmq_pybind DESTINATION .)
