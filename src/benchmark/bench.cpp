#include <fstream>
#include <iostream>
#include <raylib.h>

#include "graph/WeightedGraph.h"
#include "graph/Map.h"
#include "reference/fringe.h"
#include "reference/fringe_simd.h"
#include "ripple/RippleSearch.h"
#include "utility/FileParser.h"
#include "utility/Timer.h"

bool check_source_and_goal(Map &map, Node source, Node goal) {
  // bounds check on source and goal
  if (!map.in_bounds(map.node_to_point(source))) {
    std::cout << "Source node out of bounds" << std::endl;
    return false;
  }

  if (!map.in_bounds(map.node_to_point(goal))) {
    std::cout << "Goal node out of bounds" << std::endl;
    return false;
  }

  // check that both goal and source are not walls
  if (!map.get(map.node_to_point(source))) {
    std::cout << "Source is a wall" << std::endl;
    return false;
  }

  if (!map.get(map.node_to_point(goal))) {
    std::cout << "Goal is a wall" << std::endl;
    return false;
  }

  return true;
}

void flush_cache() {
  const int size = 128 * 1024 * 1024;
  volatile uint8_t *volatile data = (uint8_t *)malloc(size);
  for (int i = 0; i < size; i++) {
    data[i] = 0xcc;
    volatile uint8_t v = data[i];
  }

  free((void *)data);
}

template <typename Search>
void test_search(std::string name, Map &map, Node source, Node goal) {
  const int COUNT = 20;
  double min_time = 10e10, max_time = 0, avg_time = 0;
  double avg_init = 0;

  for (int i = 0; i < COUNT; i++) {
    flush_cache();

    Timer t;
    t.start();
    Search fringe(map, source, goal);
    auto shortest_path = fringe.search().value_or(Path<Node>());
    t.stop();

    double ms = t.get_microseconds() / 1000.0;
    min_time = std::min(min_time, ms);
    max_time = std::max(max_time, ms);
    avg_time += ms * (1.0 / COUNT);
  }

  printf("%s: %3.2fms avg | %3.2fms min | %3.2fms max\n", name.c_str(),
         avg_time, min_time, max_time);
}

int main(int argc, char **argv) {
  SetTraceLogLevel(LOG_NONE);

  if (argc < 4) {
    std::cout << "Usage: " << argv[0] << " PATH START GOAL" << std::endl;
    return 1;
  }

  Map map;
  std::string file_path(argv[1]);

  if (file_path.ends_with(".png")) {
    map.load_from_image_file(argv[1]);
  } else if (file_path.ends_with(".dot")) {
    WeightedGraph graph;
    DotParser(argv[1]).build_graph(graph);
    map = graph.create_map();
  } else {
    std::cout << "Unsupported file type!" << std::endl;
    return 2;
  }

  Node source = strtol(argv[2], nullptr, 10);
  Node goal = strtol(argv[3], nullptr, 10);

  if (!check_source_and_goal(map, source, goal))
    return 3;

  test_search<FringeSearch>("Fringe", map, source, goal);
  test_search<FringeSearchSimd>("Fringe Vec", map, source, goal);
}
