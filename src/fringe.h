#pragma once

#include <list>

#include "map.h"

typedef std::list<Node> FringeList;
typedef FringeList::iterator FringeEntry;

struct FringeNode {
    bool visited = false;
    bool in_list;
    int g; // Cost from source to node
    Node parent;
    FringeEntry list_entry;
};

class FringeSearch {
    FringeList fringe_list;

public:
    std::vector<FringeNode> cache;
    std::list<Node> search(Map& map, Node source, Node goal);
};