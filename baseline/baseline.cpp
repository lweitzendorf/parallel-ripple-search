#include "WeightedGraph.h"
#include "FileParser.h"

#include <iostream>

int main(int argc, char const *argv[]) {
  if (argc != 4) {
    std::cout << "Usage: ./baseline <graph_file> <source_node> <target_node>" << std::endl;
    return -1;
  }

  WeightedGraph G;
  DotParser(argv[1]).build_graph(G);

  vertex_t source = strtol(argv[2], nullptr, 10);
  vertex_t target = strtol(argv[3], nullptr, 10);

  std::cout << argv[1] << std::endl;
  std::cout << G.num_vertices() << " vertices" << std::endl;
  std::cout << G.num_edges() << " edges" << std::endl;
  std::cout << source << " -> " << target << std::endl;

  std::list<vertex_t> path = G.a_star_search(source, target);

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
