cmake_minimum_required(VERSION 3.16)
project(DPHPC)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# raylib
add_subdirectory(raylib)

# oneTBB
include(oneTBB/examples/common/cmake/common.cmake)
set_common_project_settings(tbb)

# internal
add_executable(baseline baseline/main.cpp baseline/WeightedGraph.cpp baseline/FileParser.cpp baseline/Timer.cpp src/map.cpp)
target_link_libraries(baseline raylib)

add_executable(graph_view src/graph_view.cpp src/map.cpp src/fringe.cpp baseline/Timer.cpp baseline/WeightedGraph.cpp baseline/FileParser.cpp src/Astar_high.cpp src/ripple.cpp src/high_level_path.cpp)
target_include_directories(graph_view PUBLIC baseline src)
target_link_libraries(graph_view raylib TBB::tbb Threads::Threads)

#add_executable(high_graph src/high.cpp src/map.cpp src/Astar_high.cpp)
#target_link_libraries(high_graph raylib)

#add_executable(ripple src/ripple.cpp)
#target_link_libraries(ripple TBB::tbb Threads::Threads)

