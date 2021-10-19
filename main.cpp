#include "WeightedGraph.h"

#include <iostream>
#include <fstream>
#include <cstdlib>

int main(int argc, char const *argv[]) {
  if (argc != 4) {
    std::cout << "Usage: ./ripple_search <graph_file> <start_node> <goal_node>" << std::endl;
    return -1;
  }

  const char* file_name = argv[1];
  std::ifstream graph_file(file_name);

  int x_size, y_size;
  graph_file >> x_size >> y_size;

  WeightedGraph G(x_size * y_size);
  std::vector<bool> is_path(x_size);

  int vertex_number = 0;

  for (int y = 0; y < y_size; y++) {
    for (int x = 0; x < x_size; x++) {
      bool current_is_path;
      graph_file >> current_is_path;

      G.add_location(x, y);

      if (current_is_path) {
        if (x > 0 && is_path.at(x - 1))
          G.add_edge(vertex_number - 1, vertex_number, 1);

        if (y > 0 && is_path.at(x))
          G.add_edge(vertex_number - x_size, vertex_number, 1);
      }
      is_path.at(x) = current_is_path;
      vertex_number++;
    }
  }

  graph_file.close();
  vertex_t start = strtol(argv[2], nullptr, 10), goal = strtol(argv[3], nullptr, 10);

  std::cout << argv[1] << std::endl;
  std::cout << G.num_vertices() << " vertices" << std::endl;
  std::cout << G.num_edges() << " edges" << std::endl;
  std::cout << start << " -> " << goal << std::endl;

  std::list<vertex_t> path = G.a_star_search(start, goal);

  if (path.empty()) {
    std::cout << "No path found!";
  } else {
    for (vertex_t v : path)
      std::cout << v << " ";
  }
  std::cout << std::endl;

  return 0;
}
