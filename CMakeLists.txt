cmake_minimum_required(VERSION 2.9)
project(Kitti_Read)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})


add_executable(Kitti_Read main.cpp rich_point.cpp)
target_link_libraries(Kitti_Read ${OpenCV_LIBS})
