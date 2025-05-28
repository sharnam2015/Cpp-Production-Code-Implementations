add_executable(fast fastfeatures.cpp)

#set(OpenCV_ROOT "${VCPKG_INSTALLED_DIR}/x64-windows/share/opencv4")

find_package(OpenCV REQUIRED)

#include_directories(${OpenCV_INCLUDE_DIRS})

target_include_directories(fast
PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(fast
  PRIVATE
  ${OpenCV_LIBS}
)

