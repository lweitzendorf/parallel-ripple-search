#include <filesystem>
#include <fstream>

#include "benchmarks.h"

std::vector<std::pair<std::string, WeightedGraph>> load_maps(const std::string& dir) {
  std::vector<std::pair<std::string, WeightedGraph>> result;
  for (const auto& entry : std::filesystem::directory_iterator(dir))
  {
    auto path = entry.path();
    std::fstream f(path);


    std::string trash;
    std::string type;
    int width, height;
    f >> trash >> type >> trash >> height >> trash >> width >> trash;

    WeightedGraph graph;

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        Point p(x, y);
        Node n = graph.add_vertex(p);

        char c; f >> c;
        if(c == '.') {
          for (auto offset : Map::neighbour_offsets) {
            Point neighbor = p + offset;

            if (graph.is_reachable(p) && graph.is_reachable(neighbor)) {
              Node neighbor_node = graph.point_to_vertex(neighbor).value();
              graph.add_edge(neighbor_node, n, 1);
            }
          }
        } else {
          assert(c == '@' || c == 'T');
        }
      }
    }

    result.push_back(std::make_pair(path.filename(), std::move(graph)));
  }

  return result;
}

std::vector<Scenario> load_scenarios(const std::string& dir, const std::string& map) {
  auto path = dir + map + ".scen";

  std::fstream f(path);

  std::string line;
  int bucket, width, height;
  
  std::vector<Scenario> result;

  std::string version;
  std::getline(f, version);


  while(std::getline(f, line)) {
    Scenario scen;

    std::istringstream iss(line);
    std::string name;
    iss >> bucket >> name >> width >> height >> scen.source.x >> scen.source.y >> scen.goal.x >> scen.goal.y >> scen.cost;
    assert(name == map);
    //assert(width == 512);
    //assert(height == 512);

    result.push_back(scen);
  }

  return result;
}
