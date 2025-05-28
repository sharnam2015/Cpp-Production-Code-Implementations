cmake_minimum_required(VERSION 3.10)

set(VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING "")
project(FASTfeatures LANGUAGES CXX)

# Show all warning messages
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
set(OpenCV_ROOT "${VCPKG_INSTALLED_DIR}/x64-windows/share/opencv4")

#set(CMAKE_TOOLCHAIN_FILE "c:/Users/sharn/Documents/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")

add_subdirectory(src)

