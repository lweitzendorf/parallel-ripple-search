#include "WeightedGraph.h"
#include "FileParser.h"
#include "Timer.h"

#include <iostream>

int main(int argc, char const *argv[]) {
  if (argc != 4) {
    std::cout << "Usage: ./baseline <graph_file> <source_node> <target_node>" << std::endl;
    return -1;
  }

  Timer timer;
  WeightedGraph G;
  std::cout << argv[1] << std::endl;

  timer.start();
  bool valid = DotParser(argv[1]).build_graph(G);
  timer.stop();
  std::cout << "Parsed in " << timer.get_ms() << "ms" << std::endl;

  if (!valid) {
    std::cout << "Invalid file!" << std::endl;
    return -2;
  }

  std::cout << std::endl;
  std::cout << G.num_vertices() << " vertices" << std::endl;
  std::cout << G.num_edges() << " edges" << std::endl << std::endl;

  vertex_t source = strtol(argv[2], nullptr, 10);
  vertex_t target = strtol(argv[3], nullptr, 10);

  std::cout << source << " -> " << target << std::endl;
  timer.start();
  std::list<vertex_t> path = G.a_star_search(source, target);
  timer.stop();
  std::cout << "Found path in " << timer.get_ms() << "ms" << std::endl;

  if (path.empty()) {
    std::cout << "No path found!";
  } else {
    for (vertex_t v : path) {
      std::cout << v << " ";
    }
  }
  std::cout << std::endl;

  return 0;
}
