#include <iostream>
#include <liblsb.h>
#include <string>

#include "graph/Map.h"
#include "reference/FringeSearchSimd.h"
#include "ripple/RippleSearch.h"
#include "benchmark/benchmarks.h"
#include "reference/Astar.h"
#include "reference/BoostAStarSearch.h"

#define RUNS_PER_SCENARIO 30

template <typename Search>
void benchmark_scenario(int index, Search &search, Node source, Node goal) {
  for (int i = 0; i < RUNS_PER_SCENARIO; i++) {
    LSB_Res();
    auto path = search.search(source, goal);
    LSB_Rec(index);
    LSB_Reg_param("length = %d", path.value().size());
  }
}

template <typename Search>
void benchmark(const std::string &name, std::vector<std::pair<std::string, Map>> &maps,
               std::vector<std::vector<Scenario>> &scenarios) {
  LSB_Init(name.c_str(), 0);
  int benchmark_index = 0;

  for (int i = 0; i < maps.size(); i++) {
    LSB_Reg_param("map = %s", maps[i].first.c_str());
    std::cout << "Map " << i+1 << "/" << scenarios.size() << ": " << maps[i].first << " - " ;
    std::cout << scenarios[i].size() << " benchmarks" << std::endl;
    Map &map = maps[i].second;
    Search search(map);

    for (int j = 0; j < scenarios[i].size(); j++) {
      Scenario &scenario = scenarios[i][j];
      Node source_node = map.point_to_node(scenario.source);
      Node goal_node = map.point_to_node(scenario.goal);

      std::cout << j+1 << "/" << scenarios[i].size() << ": ";
      std::cout << source_node << " -> " << goal_node;
      std::cout << ", cost = " << scenario.cost << std::endl;

      LSB_Reg_param("scenario = [%d, %d]", source_node, goal_node);
      benchmark_scenario<Search>(benchmark_index++, search, source_node, goal_node);
    }
    std::cout << std::endl;
  }

  LSB_Finalize();
}

int main() {
  std::string bench = "sc1";
  std::vector<std::pair<std::string, Map>> maps = load_maps("../benchmarks/" + bench + "-map");
  std::cout << "Benchmark: " << bench <<" - loaded " << maps.size() << " maps" << std::endl;

  std::vector<std::vector<Scenario>> scenarios;

  for(auto &m : maps) {
    auto scen = load_scenarios("../benchmarks/" + bench + "-scen/", m.first);
    scenarios.push_back(std::move(scen));
  }

  benchmark<RippleSearch>("ripple-"+std::to_string(NUM_THREADS), maps, scenarios);
  benchmark<FringeSearch>("fringe", maps, scenarios);
  benchmark<FringeSearchSimd>("fringe-simd", maps, scenarios);
  benchmark<AStarSearch>("a-star", maps, scenarios);
  benchmark<BoostAStarSearch>("boost-a-star", maps, scenarios);
}
