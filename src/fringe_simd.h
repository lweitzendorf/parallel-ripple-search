#pragma once

#include <list>
#include <stdint.h>
#include <optional>

#include "map.h"

struct FringeNodeSimd {
    // Replace with flags
    bool visited = false;
    char list_index = -1;
    bool phase2 = false;

    float g; // Cost from source to node
    Node parent;
};

class FringeSearchSimd {
    // Constants
    Node source;
    Node goal;
    Map& map;

    // Updated by step
    std::vector<Node> now_list;
    std::vector<Node> later_list;
    
public:
    std::vector<FringeNodeSimd> cache;

    FringeSearchSimd(Map& map, Node source, Node goal);
    std::optional<std::vector<Node>> search();
};
