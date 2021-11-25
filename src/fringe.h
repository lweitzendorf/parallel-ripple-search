#pragma once

#include <list>

#include "map.h"

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


enum class FringeSearchStepState {
    OK,
    UNREACHABLE,
    FOUND,
};

struct FringeSearchStep {
    FringeSearchStepState state;

};

class FringeSearch {
    // Constants
    Node source;
    Node goal;
    Map& map;

    // Updated by step
    FringeList fringe_list;
    FringeEntry nnode;
    int flimit;
    int fmin;

    
public:
    std::vector<FringeNode> cache;
    std::list<Node> search();

    FringeSearch(Map& map);
    void init(Node source, Node goal);
    FringeSearchStep step();
    std::list<Node> finalize_path();
};
