cmake_minimum_required(VERSION 3.10)
project(FilterProject LANGUAGES CXX)

# Show all warning messages
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

enable_testing()

add_subdirectory(src)
add_subdirectory(tests)
