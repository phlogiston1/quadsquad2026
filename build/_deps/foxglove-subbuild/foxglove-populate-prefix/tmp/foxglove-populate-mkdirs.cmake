# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-src"
  "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-build"
  "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-subbuild/foxglove-populate-prefix"
  "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-subbuild/foxglove-populate-prefix/tmp"
  "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-subbuild/foxglove-populate-prefix/src/foxglove-populate-stamp"
  "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-subbuild/foxglove-populate-prefix/src"
  "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-subbuild/foxglove-populate-prefix/src/foxglove-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-subbuild/foxglove-populate-prefix/src/foxglove-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/seanb/Documents/quadsquad2026/build/_deps/foxglove-subbuild/foxglove-populate-prefix/src/foxglove-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
