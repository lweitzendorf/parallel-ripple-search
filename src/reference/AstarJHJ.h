#pragma once

#include "graph/Map.h"
#include "stlastar.h"
#include <optional>

class AstarJHJ {
  // Constants
  Map &map;

  class MapSearchNode {
  public:
    Map *map;
    Node pos;

    MapSearchNode() : map(NULL) {}
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

  AstarJHJ(Map &map) : map(map){};
};
