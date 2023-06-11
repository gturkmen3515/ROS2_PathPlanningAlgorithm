project(untitled)
cmake_minimum_required(VERSION 3.25)
set(CMAKE_CXX_STANDARD 17)

find_package(Python3 COMPONENTS Intrepreter Development NumPy)
include_directories(${Python3_INCLUDE_DIRS} include)

add_executable(result_plot main.cpp matplotlibcpp.h astar.h)
target_link_libraries(result_plot ${Python3_LIBRARIES})
