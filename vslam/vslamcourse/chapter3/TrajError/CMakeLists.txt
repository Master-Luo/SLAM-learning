cmake_minimum_required(VERSION 3.10)
project(traerror)

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(trajectoryError trajectoryError.cpp)

target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})
target_link_libraries(trajectoryError Sophus::Sophus)
