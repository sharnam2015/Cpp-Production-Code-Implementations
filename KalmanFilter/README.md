# Implemented a Kalman Filter in production style C++ code
The Kalman Filter implementation is done using basic data structures like arrays and then overloading operators so that these arrays can behave like matrices, all the way to implementing a little more robust version of the kalman filter to writing unit tests for each functionality in the tests folder

## How to Run (Build and Test)
- cmake --build . --config Debug --parallel
- ctest --output-on-failure -C Debug 
