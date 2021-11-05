#include "WeightedGraph.h"
#include "FileParser.h"

#include <iostream>
#include <cstdlib>

int main(int argc, char const *argv[]) {
  if (argc != 4) {
    std::cout << "Usage: ./a_star <graph_file> <start_node> <goal_node>" << std::endl;
    return -1;
  }

  WeightedGraph G;
  DotParser(argv[1]).build_graph(G);

  vertex_t start = strtol(argv[2], nullptr, 10);
  vertex_t goal = strtol(argv[3], nullptr, 10);

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
