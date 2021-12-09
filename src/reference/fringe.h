#pragma once

#include <list>
#include <optional>
#include "CLionProjects/parallel-ripple-search/src/graph/map.h"

typedef std::list<Node> FringeList;
typedef FringeList::iterator FringeEntry;

struct FringeNode {
    // Replace with flags
    bool visited = false;
    bool in_list = false;
    bool phase2 = false;

    int g; // Cost from source to node
    Node parent;
    FringeEntry list_entry;
};

class FringeSearch {
    // Constants
    Node source;
    Node goal;
    Map& map;
    FringeList fringe_list;
public:
    std::vector<FringeNode> cache;
    std::optional<std::vector<Node>> search();

    FringeSearch(Map& map, Node source, Node goal);
};
