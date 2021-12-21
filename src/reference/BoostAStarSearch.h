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
    auto list = graph.a_star_search(source, goal);

    if (list.empty())
      return {};

    std::vector<Node> path;
    std::copy(list.begin(), list.end(), std::back_inserter(path));
    return path;
  }
};