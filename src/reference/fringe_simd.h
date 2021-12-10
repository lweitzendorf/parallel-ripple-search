#pragma once

#include <list>
#include <optional>
#include <stdint.h>

#include "graph/map.h"

struct FringeNodeSimd {
  Node parent;
  float g; // Cost from source to node
  int32_t visited = 0;
  int32_t list_index = -1;
};

static_assert(sizeof(FringeNodeSimd) == 4 * 4);

class FringeSearchSimd {
  // Constants
  Node source;
  Node goal;
  Map &map;

  // Updated by step
  std::vector<Node> now_list;
  std::vector<Node> later_list;

public:
  std::vector<FringeNodeSimd> cache;

  FringeSearchSimd(Map &map, Node source, Node goal);
  std::optional<std::vector<Node>> search();
};
