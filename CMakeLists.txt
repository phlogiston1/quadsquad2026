cmake_minimum_required(VERSION 3.10)
project(quadsquad2026)

include(FetchContent)
FetchContent_Declare(
    foxglove
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    URL https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.14.3/foxglove-v0.14.3-cpp-x86_64-pc-windows-msvc.zip
    URL_HASH SHA256=a7d1deea49d86ba472d817a7eda507190969d6bd32041ae7441a82bade3afb45
)
FetchContent_MakeAvailable(foxglove)

file(GLOB CONTROL_SOURCES "src/*.cpp")
add_executable(simulator ${CONTROL_SOURCES})

target_include_directories(simulator PRIVATE
    ${foxglove_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(simulator PRIVATE
    ${foxglove_SOURCE_DIR}/lib/foxglove.lib
)

add_custom_command(TARGET simulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${foxglove_SOURCE_DIR}/lib/foxglove.dll"
        $<TARGET_FILE_DIR:simulator>)
