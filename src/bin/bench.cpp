#include <fstream>
#include <iostream>
#include <raylib.h>
#include <filesystem>

#include "graph/WeightedGraph.h"
#include "graph/Map.h"
#include "reference/FringeSearch.h"
#include "reference/FringeSearchSimd.h"
#include "ripple/RippleSearch.h"
#include "utility/FileParser.h"
#include "utility/Timer.h"
#include "reference/Astar.h"
#include "ripple/HighLevelGraph.h"
#include "benchmark/benchmarks.h"

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
void benchmark_search(std::string name, Map &map, Node source, Node goal) {
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
  //std::string bench = "bg512";
  std::string bench = "sc1";
  auto maps = load_maps("../benchmarks/" + bench + "-map");
  std::cout << "Benchmark: " << bench <<" - loaded " << maps.size() << " maps" << std::endl;

  std::vector<std::vector<Scenario>> scenarios;

  for(auto &m : maps) {
    auto& map = m.second;
    auto scen = load_scenarios("../benchmarks/" + bench + "-scen/", m.first);
    scenarios.push_back(std::move(scen));
  }

  auto& m = maps.front();
  auto& map = m.second;
  auto s = scenarios.front().back();
  benchmark_search<FringeSearchSimd>("Fringe Vec", map, map.point_to_node(s.source), map.point_to_node(s.goal));
}
