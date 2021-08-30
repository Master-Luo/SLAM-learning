cmake_minimum_required(VERSION 3.10)
project(ch1)

set(CMAKE_CXX_STANDARD 11)

include_directories( "/usr/include/eigen3" )
add_executable(ch1 Eigen.cpp)
