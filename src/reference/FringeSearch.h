#pragma once

#include "graph/Map.h"
#include <list>
#include <optional>

typedef std::list<Node> FringeList;
typedef FringeList::iterator FringeEntry;

struct FringeNode {
  // Replace with flags
  bool visited = false;
  bool in_list = false;
  bool phase = false;

  int cost; // Cost from source to node
  Node parent = -34; // TODO losco
  FringeEntry list_entry;
};

class FringeSearch {
  // Constants
  Map &map;
  FringeList fringe_list;

public:
  std::vector<FringeNode> cache;
  std::optional<std::vector<Node>> search(Node source, Node goal);

  FringeSearch(Map &map);
};
