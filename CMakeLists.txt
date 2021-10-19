cmake_minimum_required(VERSION 3.16)
project(DPHPC)

set(CMAKE_CXX_STANDARD 20)

add_executable(ripple_search main.cpp WeightedGraph.cpp WeightedGraph.h)
add_executable(graph_generator graph_generator.cpp)