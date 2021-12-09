cmake_minimum_required(VERSION 3.16)
project(DPHPC)

set(CMAKE_CXX_STANDARD 20)

# raylib
add_subdirectory(raylib)
set(PLATFORM=PLATFORM_DESKTOP)
set(GRAPHICS=GRAPHICS_API_OPENGL_11)

# oneTBB
include(oneTBB/examples/common/cmake/common.cmake)
set_common_project_settings(tbb)

# NOTE uncomment the below definition if raylib raises SIGSEV
# add_definitions(-DONLY_EXPORT_IMGS)
# NOTE we should turn this on at some point
# add_compile_options(-Wall -Werror)

# Set to TRUE to make a debug build
set(DEBUG FALSE)
# Set to TRUE to enable avx512 vectorization
set(AVX512 FALSE)

if (DEBUG)
    set(CMAKE_BUILD_TYPE Debug)
else ()
    set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif ()


if(AVX512)
    add_compile_definitions(AVX512_ENABLED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=skylake-avx512")
endif ()

set(BaselineSources src/WeightedGraph.cpp src/Timer.cpp src/FileParser.cpp)
set(RippleSources src/high_level_path.cpp src/fringe_simd.cpp src/fringe.cpp src/ripple.cpp src/map.cpp)
add_executable(graph_view src/graph_view.cpp ${RippleSources} ${BaselineSources})
target_include_directories(graph_view PUBLIC baseline src)
target_link_libraries(graph_view raylib TBB::tbb Threads::Threads)

add_executable(bench src/bench.cpp ${RippleSources} ${BaselineSources})
target_include_directories(bench PUBLIC baseline src)
target_link_libraries(bench raylib TBB::tbb Threads::Threads)
