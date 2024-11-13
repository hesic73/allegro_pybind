cmake_minimum_required(VERSION 3.15)
project(AllegroHand LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable pybind11's automatic Python detection
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)  # Find Eigen3

# Define the source files
set(SOURCES src/main.cpp src/canAPI.cpp src/allegro_interface.cpp)

# Add the include directory for headers
include_directories(include)

# Create the Python module with the desired name
pybind11_add_module(allegro_pybind ${SOURCES})

# Link required libraries
target_link_libraries(allegro_pybind PRIVATE BHand pcanbasic Eigen3::Eigen)

# Install the module to the current directory for Python import
install(TARGETS allegro_pybind DESTINATION .)
