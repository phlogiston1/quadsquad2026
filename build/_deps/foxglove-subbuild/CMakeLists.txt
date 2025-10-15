# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.28.3)

# Reject any attempt to use a toolchain file. We must not use one because
# we could be downloading it here. If the CMAKE_TOOLCHAIN_FILE environment
# variable is set, the cache variable will have been initialized from it.
unset(CMAKE_TOOLCHAIN_FILE CACHE)
unset(ENV{CMAKE_TOOLCHAIN_FILE})

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(foxglove-populate NONE)



include(ExternalProject)
ExternalProject_Add(foxglove-populate
                     "UPDATE_DISCONNECTED" "False" "DOWNLOAD_EXTRACT_TIMESTAMP" "TRUE" "EXTERNALPROJECT_INTERNAL_ARGUMENT_SEPARATOR" "URL" "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.14.3/foxglove-v0.14.3-cpp-x86_64-unknown-linux-gnu.zip" "URL_HASH" "SHA256=68efcf9b2cb2c94d37af77528748e13d3af548340539f70d7afffa2406d3b5e1"
                    SOURCE_DIR          "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-src"
                    BINARY_DIR          "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
                    USES_TERMINAL_PATCH     YES
)


