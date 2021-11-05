cmake_minimum_required(VERSION 3.16)
project(DPHPC)

set(CMAKE_CXX_STANDARD 20)

add_executable(baseline baseline/baseline.cpp baseline/WeightedGraph.cpp baseline/WeightedGraph.h baseline/FileParser.cpp baseline/FileParser.h)

add_subdirectory(raylib)
add_executable(graph_view graph_view.cpp)
add_executable(high_graph high.cpp)
target_link_libraries(graph_view raylib)
target_link_libraries(high_graph raylib)


include(oneTBB/examples/common/cmake/common.cmake)
set_common_project_settings(tbb)

add_executable(ripple ripple.cpp)
target_link_libraries(ripple TBB::tbb Threads::Threads)

