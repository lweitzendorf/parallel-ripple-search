cmake_minimum_required(VERSION 3.16)
project(DPHPC)

set(CMAKE_CXX_STANDARD 20)

add_executable(a_star main.cpp WeightedGraph.cpp WeightedGraph.h)

add_executable(graph_generator graph_generator.cpp)

add_subdirectory(raylib)
add_executable(graph_view graph_view.cpp)
target_link_libraries(graph_view raylib)
