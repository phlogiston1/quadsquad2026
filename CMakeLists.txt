# Set the minimum version of CMake that can be used
# To find the cmake version run
# $ cmake --version
cmake_minimum_required(VERSION 3.10)

# Set the project name
project (quadsquad2025)

# Create a sources variable with a link to all cpp files to compile
file(GLOB CONTROL_SOURCES "src/*.cpp")

# Add an executable with the above sources
add_executable(simulator ${CONTROL_SOURCES})

# Set the directories that should be included in the build command for this target
# when running g++ these will be included as -I/directory/path/
target_include_directories(simulator
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)