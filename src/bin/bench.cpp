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

float compute_path_cost(Map& map, Path<Node>& path) {
  float total_cost = 0;
  for(size_t i = 0; i < path.size() - 1; i++) {
    Point p1 = map.node_to_point(path[i]);
    Point p2 = map.node_to_point(path[i + 1]);
    float cost = p1.x == p2.x || p1.y == p2.y ? 1 : sqrtf(2.0f);
    total_cost += cost;
  }

  return total_cost;
}

template <typename Search>
void benchmark_single_search(std::string name, Map &map, Node source, Node goal) {
  const int COUNT = 1;
  double min_time = 10e10, max_time = 0, avg_time = 0;
  double avg_init = 0;

  for (int i = 0; i < COUNT; i++) {
    //flush_cache();

    Timer t;
    t.start();
    Search fringe(map, source, goal);
    auto shortest_path = fringe.search().value_or(Path<Node>());
    t.stop();

    double ms = t.get_nanoseconds() / 1e6;
    min_time = std::min(min_time, ms);
    max_time = std::max(max_time, ms);
    avg_time += ms * (1.0 / COUNT);
  }

  printf("%s: %3.2fms avg | %3.2fms min | %3.2fms max\n", name.c_str(),
         avg_time, min_time, max_time);
}


template <typename Search>
void benchmark_all_scenarios(const std::string& name, Map &map, std::vector<Scenario>& scenarios, FILE* f) 
{
  const int COUNT = 1;
  double min_time = 10e10, max_time = 0, avg_time = 0;
  double avg_init = 0;

  float delta_cost = 0;
  float min_delta = FLT_MAX;
  
  Search search(map);
  for (int i = 0; i < COUNT; i++) {
    uint64_t total_time = 0;

    for(size_t j = scenarios.size() / 2; j < scenarios.size(); j += 1) {
      Scenario scen = scenarios[j];
      Node s = map.point_to_node(scen.source);
      Node g = map.point_to_node(scen.goal);
      
      Timer t;
      t.start();
      auto path_opt = search.search(s, g);
      t.stop();
      total_time += t.get_nanoseconds();

      if(!path_opt.has_value()) {
        printf("Error\n");
        exit(1);
      }

      if(i == 0) {
        float cost = compute_path_cost(map, path_opt.value());
        float delta =(cost - scen.cost);
        //printf("%4d - %4.2f delta\n", (int)j, delta);
        delta_cost += delta / scenarios.size();
        min_delta = std::min(min_delta, delta);
      }
    }

    double ms = total_time / 1e6;
    min_time = std::min(min_time, ms);
    max_time = std::max(max_time, ms);
    avg_time += ms * (1.0 / COUNT);
  }

  printf("%-10s: %7.2fms avg | %7.2fms min | %7.2fms max | %5.2f min delta cost | %5.2f avg delta cost\n", name.c_str(),
         avg_time, min_time, max_time, min_delta, delta_cost);

  fprintf(f, "%10.4f %10.4f\n", avg_time, delta_cost);
  fflush(f);
}

class AstarSearch {
  Map& map;

public:
  AstarSearch(Map& map) : map(map) {}
  std::optional<Path<Node>> search(Node source, Node goal) { return a_star_search(map, source, goal); }
};

class BoostSearch {
  WeightedGraph graph;

public:
  BoostSearch(Map& map) {
    graph.build_from_map(map);
  }

  std::optional<Path<Node>> search(Node source, Node goal) { return graph.a_star_search(source, goal); }
};


int main(int argc, char **argv) {
  std::string bench = "bg512";
  //std::string bench = "sc1";
  auto maps = load_maps("../benchmarks/" + bench + "-map");
  //std::cout << "Benchmark: " << bench <<" - loaded " << maps.size() << " maps" << std::endl;

  std::vector<std::vector<Scenario>> scenarios;

  for(auto &m : maps) {
    auto& map = m.second;
    auto scen = load_scenarios("../benchmarks/" + bench + "-scen/", m.first);
    scenarios.push_back(std::move(scen));
  }


  FILE* f_ripple = fopen("sc1b_ripple.dat", "w");
  FILE* f_fringe_vec = fopen("sc1b_fringe_vec.dat", "w");
  FILE* f_fringe = fopen("sc1b_fringe.dat", "w");
  FILE* f_astar = fopen("sc1b_astar.dat", "w");
  FILE* f_boost = fopen("sc1b_boost.dat", "w");

  //int map_index = atoi(argv[1]);

  for(int map_index = 0; map_index < (int)maps.size(); map_index++)
  {
    auto& m = maps[map_index];
    auto& map = m.second;
    auto s = scenarios[map_index];
    printf("Benchmarking %s (%d - %d x %d) with %d scenarios:\n", m.first.c_str(), map_index, map.width(), map.height(), (int)s.size());
    benchmark_all_scenarios<RippleSearch>("Ripple", map, s, f_ripple);
    //benchmark_all_scenarios<FringeSearchSimd>("Fringe Vec", map, s, f_fringe_vec);
    //benchmark_all_scenarios<FringeSearch>("Fringe", map, s, f_fringe);
    //benchmark_all_scenarios<AstarSearch>("Astar", map, s, f_astar);
    //benchmark_all_scenarios<BoostSearch>("Boost", map, s, f_boost);
  }

  fclose(f_ripple);
  fclose(f_fringe_vec);
  fclose(f_fringe);
  fclose(f_astar);
  fclose(f_boost);
}
