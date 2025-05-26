cmake_minimum_required(VERSION 3.10)
enable_testing()

find_package(GTest CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_executable(FilterTests FilterTests.cpp ../src/KalmanFilter.cpp)

target_link_libraries(FilterTests
  PRIVATE
    GTest::gtest_main
    Threads::Threads
)

add_test(NAME FilterTests COMMAND FilterTests)
