cmake_minimum_required(VERSION 3.10)
set(PACKAGE_NAME RRT)

project(${PACKAGE_NAME} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
set(dependencies
    OpenCV
)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(
  include
)

add_executable(${PACKAGE_NAME}_main ./rrt_cv.cpp)
target_link_libraries(${PACKAGE_NAME}_main ${OpenCV_LIBS} )