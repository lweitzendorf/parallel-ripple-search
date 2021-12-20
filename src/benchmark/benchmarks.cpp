#include <filesystem>
#include <fstream>

#include "benchmarks.h"

std::vector<std::pair<std::string, Map>> load_maps(const std::string& dir) {
  std::vector<std::pair<std::string, Map>> result;
  for (const auto& entry : std::filesystem::directory_iterator(dir))
  {
    auto path = entry.path();
    std::fstream f(path);


    std::string trash;
    std::string type;
    int width, height;
    f >> trash >> type >> trash >> height >> trash >> width >> trash;

    Map map(width, height);

    for(int y = 0; y < height; y++) {
      for(int x = 0; x < width; x++) {
        char c; f >> c;
        if(c == '@' || c == 'T') {
          map.set(Point(x, y), 0);
        } else if(c == '.') {
          map.set(Point(x, y), 1);
        } else {
          assert(false);
        }
      }
    }

    result.push_back(std::make_pair(path.filename(), std::move(map)));
  }

  std::sort(result.begin(), result.end(), [](auto &p1, auto &p2) -> bool { return p1.first < p2.first; });
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
