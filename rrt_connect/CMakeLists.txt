cmake_minimum_required(VERSION 3.0)
project(planner)
# set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

include_directories(include)

add_executable(planner main.cpp)
target_link_libraries(planner PRIVATE ${OpenCV_LIBRARIES})