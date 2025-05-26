add_executable(Kalman main.cpp KalmanFilter.cpp)

target_include_directories(Kalman 
PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}
)


