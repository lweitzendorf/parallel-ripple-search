#pragma once

#include "AStarJHJ.h"
#include "graph/Map.h"
#include <optional>

class JHJAstarSearch {
  // Constants
  Map &map;

  class MapSearchNode {
  public:
    Map *map;
    Node pos;

    MapSearchNode() : map(nullptr) {}
    MapSearchNode(Map *map, Node n) : map(map), pos(n) {}

    float GoalDistanceEstimate(MapSearchNode &nodeGoal);
    bool IsGoal(MapSearchNode &nodeGoal);
    bool GetSuccessors(AStarJHJSearch<MapSearchNode> *astarsearch,
                       MapSearchNode *parent_node);
    float GetCost(MapSearchNode &successor);
    bool IsSameState(MapSearchNode &rhs);
    int Hash();
    void PrintNodeInfo();
  };

public:
  std::optional<std::vector<Node>> search(Node source, Node goal);

  JHJAstarSearch(Map &map) : map(map){};
};
