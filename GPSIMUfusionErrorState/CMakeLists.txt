cmake_minimum_required(VERSION 3.10)

set(VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING "")
project(Fusion LANGUAGES CXX)

# Show all warning messages
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

add_subdirectory(src)
