#include <iostream>
#include <liblsb.h>
#include <string>

#include "graph/Map.h"
#include "reference/FringeSearchSimd.h"
#include "ripple/RippleSearch.h"
#include "benchmark/benchmarks.h"
#include "reference/Astar.h"
#include "reference/BoostAStarSearch.h"

#define RUNS_PER_SCENARIO 20

template <typename Search>
void benchmark_scenario(int index, Map &map, Node source, Node goal) {
  for (int i = 0; i < RUNS_PER_SCENARIO; i++) {
    Search search(map, source, goal);
    LSB_Res();
    auto shortest_path = search.search().value_or(Path<Node>());
    LSB_Rec(index);
    //LSB_Reg_param("%ld\n", shortest_path.size());
  }
}

template <typename Search>
void benchmark(std::string name, std::vector<std::pair<std::string, Map>> &maps,
                                 std::vector<std::vector<Scenario>> &scenarios) {
  LSB_Init(name.c_str(), 0);
  int benchmark_index = 0;

  for (int i = 0; i < scenarios.size(); i++) {
    std::cout << "Map " << i+1 << "/" << scenarios.size() << ": " << maps[i].first << " - " ;
    std::cout << scenarios[i].size() << " benchmarks" << std::endl;
    Map &map = maps[i].second;

    for (int j = 0; j < scenarios[i].size(); j+=50) {
      Scenario &scenario = scenarios[i][j];
      Node source_node = map.point_to_node(scenario.source);
      Node goal_node = map.point_to_node(scenario.goal);

      std::cout << j+1 << "/" << scenarios[i].size() << ": ";
      std::cout << source_node << " -> " << goal_node;
      std::cout << ", cost = " << scenario.cost << std::endl;

      //LSB_Reg_param("Optimal cost: %f\n", scenario.cost);
      benchmark_scenario<Search>(benchmark_index++, map, source_node, goal_node);
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
