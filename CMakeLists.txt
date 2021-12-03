cmake_minimum_required(VERSION 3.16)
project(DPHPC)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# raylib
add_subdirectory(raylib)
set(PLATFORM=PLATFORM_DESKTOP)
set(GRAPHICS=GRAPHICS_API_OPENGL_11)

# oneTBB
include(oneTBB/examples/common/cmake/common.cmake)
set_common_project_settings(tbb)

# internal
add_executable(baseline baseline/main.cpp baseline/WeightedGraph.cpp baseline/FileParser.cpp baseline/Timer.cpp src/map.cpp)
target_link_libraries(baseline raylib)

# NOTE uncomment the below definition if raylib raises SIGSEV
# add_definitions(-DONLY_EXPORT_IMGS)
# NOTE we should turn this on at some point
# add_compile_options(-Wall -Werror)
set(BaselineSources baseline/WeightedGraph.cpp baseline/Timer.cpp baseline/FileParser.cpp)
set(RippleSources src/high_level_path.cpp src/fringe_simd.cpp src/fringe.cpp src/ripple.cpp src/map.cpp)
add_executable(graph_view src/graph_view.cpp ${RippleSources} ${BaselineSources})
target_include_directories(graph_view PUBLIC baseline src)
target_link_libraries(graph_view raylib TBB::tbb Threads::Threads)

add_executable(bench src/bench.cpp ${RippleSources} ${BaselineSources})
target_include_directories(bench PUBLIC baseline src)
target_link_libraries(bench raylib TBB::tbb Threads::Threads)
