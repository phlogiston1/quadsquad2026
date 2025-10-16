cmake_minimum_required(VERSION 3.10)
project(quadsquad2026)

include(FetchContent)
FetchContent_Declare(
    foxglove
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    URL https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.14.3/foxglove-v0.14.3-cpp-x86_64-unknown-linux-gnu.zip
    URL_HASH SHA256=68efcf9b2cb2c94d37af77528748e13d3af548340539f70d7afffa2406d3b5e1
)
FetchContent_MakeAvailable(foxglove)

file(GLOB SIMULATE_SOURCES "src/QCLib/*.cpp" "src/simulate.cpp")
add_executable(simulate ${SIMULATE_SOURCES})

target_include_directories(simulate PRIVATE
    ${foxglove_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include
)

# Add include directory for Foxglove SDK
target_include_directories(simulate PRIVATE ${foxglove_SOURCE_DIR}/include)
target_include_directories(simulate PRIVATE ${foxglove_SOURCE_DIR}/include/foxglove)

# Find all Foxglove SDK source files
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
    "${foxglove_SOURCE_DIR}/src/*.cpp"
    "${foxglove_SOURCE_DIR}/src/server/*.cpp"
)

# Add Foxglove SDK source files
target_sources(simulate PRIVATE ${FOXGLOVE_SOURCES})

target_link_libraries(simulate PRIVATE
    ${foxglove_SOURCE_DIR}/lib/libfoxglove.a
)

# add_custom_command(TARGET simulate POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different
#         "${foxglove_SOURCE_DIR}/lib/libfoxglove.a"
#         $<TARGET_FILE_DIR:simulate>)
