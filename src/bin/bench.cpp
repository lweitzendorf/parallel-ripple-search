#include <fstream>
#include <iostream>
#include <raylib.h>
#include <filesystem>
#include <liblsb.h>

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

#define MIN_COST 300
#define SCENARIOS_PER_MAP 50
#define RUNS_PER_SCENARIO 20

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
void benchmark_search(int index, Map &map, Node source, Node goal) {
  for (int i = 0; i < RUNS_PER_SCENARIO; i++) {
    flush_cache();

    LSB_Res();
    Search search(map, source, goal);
    auto shortest_path = search.search().value_or(Path<Node>());
    LSB_Rec(index);
    LSB_Reg_param("%ld\n", shortest_path.size());
  }
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

  LSB_Init("ripple", 0);
  int benchmark_index = 0;

  for (int i = 0; i < scenarios.size(); i++) {
    std::cout << "Map " << i+1 << "/" << scenarios.size() << ": " << maps[i].first << " - " ;
    std::cout << scenarios[i].size() << " benchmarks" << std::endl;
    Map &map = maps[i].second;

    int j = 0;
    while (scenarios[i][j++].cost < MIN_COST);
    size_t step_size = (scenarios[i].size() - j) / SCENARIOS_PER_MAP;

    for (; j < scenarios[i].size(); j+=step_size) {
      Scenario &scenario = scenarios[i][j];
      Node source_node = map.point_to_node(scenario.source);
      Node goal_node = map.point_to_node(scenario.goal);

      std::cout << j+1 << "/" << scenarios[i].size() << ": ";
      std::cout << source_node << " -> " << goal_node << std::endl;

      LSB_Reg_param("Optimal cost: %f\n", scenario.cost);
      benchmark_search<RippleSearch>(benchmark_index++, map, source_node, goal_node);
    }
    std::cout << std::endl;
  }

  LSB_Finalize();


}
