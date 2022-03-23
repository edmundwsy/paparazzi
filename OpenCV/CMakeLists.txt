cmake_minimum_required(VERSION 3.0.0)
project(depth_est VERSION 0.1.0)

find_package(OpenCV REQUIRED)

add_executable(depth_est main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_est ${OpenCV_LIBS} ${CMAKE_THREADS_LIB_INIT})