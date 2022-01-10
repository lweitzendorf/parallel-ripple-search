#pragma once

#include <list>
#include <optional>
#include <stdint.h>

#include "graph/Map.h"

struct FringeNodeVec {
  Node parent;
  float g; // Cost from source to node
  int32_t visited = 0;
  int32_t list_index = -1;
};

static_assert(sizeof(FringeNodeVec) == 4 * 4);

class FringeSearchVec {
  // Constants
  Map &map;

  // Updated by step
  std::vector<Node> now_list;
  std::vector<Node> later_list;

public:
  std::vector<FringeNodeVec> cache;

  FringeSearchVec(Map &map);
  std::optional<std::vector<Node>> search(Node source, Node goal);
};
