#pragma once

#include "graph/WeightedGraph.h"

class BoostAStarSearch {
private:
  WeightedGraph graph;
  vertex_t source;
  vertex_t goal;

public:
  BoostAStarSearch(Map &map, Node source, Node goal) : source(source), goal(goal) {
    graph.build_from_map(map);
  }

  std::optional<Path<Node>> search() {
    return graph.a_star_search(source, goal);
  }
};