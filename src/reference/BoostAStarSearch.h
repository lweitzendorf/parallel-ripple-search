#pragma once

#include "graph/WeightedGraph.h"

class BoostAStarSearch {
  private:
  WeightedGraph graph;

  public:
  BoostAStarSearch(Map &map) {
    graph.build_from_map(map);
  }

  std::optional<Path<Node>> search(Node source, Node goal) {
    return graph.a_star_search(source, goal);
  }
};