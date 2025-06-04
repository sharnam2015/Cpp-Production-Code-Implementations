add_executable(gpsimufuse fusion.cpp main.cpp)

find_package(Eigen3 CONFIG REQUIRED)

target_include_directories(gpsimufuse
PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(gpsimufuse
  PRIVATE
  Eigen3::Eigen
)

