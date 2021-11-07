cmake_minimum_required(VERSION 3.16)
project(DPHPC)

set(CMAKE_CXX_STANDARD 20)
#set(CMAKE_BUILD_TYPE Debug)

add_executable(baseline baseline/baseline.cpp baseline/WeightedGraph.cpp baseline/WeightedGraph.h baseline/FileParser.cpp baseline/FileParser.h baseline/Timer.cpp baseline/Timer.h)

add_subdirectory(raylib)

add_executable(graph_view graph_view.cpp)
add_executable(high_graph high.cpp)
add_executable(fringe fringe.cpp)

target_link_libraries(graph_view raylib)
target_link_libraries(high_graph raylib)
target_link_libraries(fringe raylib)


include(oneTBB/examples/common/cmake/common.cmake)
set_common_project_settings(tbb)

add_executable(ripple ripple.cpp)
target_link_libraries(ripple TBB::tbb Threads::Threads)

