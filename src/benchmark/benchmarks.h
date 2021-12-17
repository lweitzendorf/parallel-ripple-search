#include "graph/Map.h"

struct Scenario {
  Point source;
  Point goal;
  float cost;
};

// Load all maps in a directory and return a vector with an entry for each map 
// consisting of full name of the map file and the map itself.
std::vector<std::pair<std::string, Map>> load_maps(const std::string& dir);

// Load all scenarios for a specific map in the specified directory.
// 'map' must be the name that was returned by load_maps
std::vector<Scenario> load_scenarios(const std::string& dir, const std::string& map);