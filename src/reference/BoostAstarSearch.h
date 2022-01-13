#pragma once

#include "graph/WeightedGraph.h"

class BoostAstarSearch {
  private:
  WeightedGraph graph;

  public:
  BoostAstarSearch(Map &map) {
    graph.build_from_map(map);
  }

  std::optional<Path<Node>> search(Node source, Node goal) {
    return graph.a_star_search(source, goal);
  }
};