#include "FringeSearch.h"

#include <limits.h>

#include <stdio.h>

std::optional<std::vector<Node>> FringeSearch::search() {
    cache.clear();
    cache.resize(map.size(), {});

    fringe_list.clear();

    // Add source node to fringe list
    fringe_list.push_front(source);
    cache[source].in_list = true;
    cache[source].g = 0;
    cache[source].parent = source;
    cache[source].list_entry = fringe_list.begin();
    cache[source].visited = true;

    // Lambda for computing heuristic
    auto h = [&](Node i) {
        return map.distance(i, goal);
    };

    int flimit = h(source);
    int found = false;
    FringeEntry nnode = fringe_list.end();

    // int it = 0;
    while(!found && !fringe_list.empty()) {
        //std::cout << "Iteration: " << it++ << std::endl;
        int fmin = INT_MAX;
        nnode = fringe_list.begin();
        
        do {
            
            if(*nnode == goal) {
                found = true;
                break;
            }

            auto& n = cache[*nnode];

            int g = n.g;
            int f = g + h(*nnode);

            if(f > flimit) {
                fmin = std::min(fmin, f);
                nnode++;
                continue;
            }

            
            Point np = map.node_to_point(*nnode);

            // For each neighbour
            for (int i = 0; i < Map::NEIGHBOURS_COUNT; i++) {
                Point neigh = Map::neighbour_offsets[i];
                neigh.x += np.x;
                neigh.y += np.y;

                if (map.in_bounds(neigh)) {
                    Node s = map.point_to_node(neigh);

                    if(!map.get(neigh)) {
                        continue;
                    }

                    int gs = g + 1;
                    if(cache[s].visited) {
                        int gi = cache[s].g;
                        if(gs >= gi) {
                            continue;
                        }
                    }

                    if(cache[s].in_list) {
                        fringe_list.erase(cache[s].list_entry);
                        cache[s].in_list = false;
                    }

                    cache[s].visited = true;
                    cache[s].g = gs;
                    cache[s].parent = *nnode;
                    //cache[s].h = h(s);
                    
                    fringe_list.insert(std::next(nnode), s);

                    cache[s].list_entry = std::next(nnode);
                    cache[s].in_list = true;
                }
            }
            
            cache[*nnode].in_list = false;

            auto tmp = std::next(nnode);
            fringe_list.erase(nnode);
            nnode = tmp;
        } while(nnode != fringe_list.end());

        
        flimit = fmin;
    }

    if(nnode != fringe_list.end()) {
        std::vector<Node> shortest_path = { goal };
        for (Node v = cache[goal].parent; v != source; v = cache[v].parent) {
            shortest_path.push_back(v);
        }
        shortest_path.push_back(source);
        std::reverse(shortest_path.begin(), shortest_path.end());
        return std::optional<Path<Node>>{shortest_path};
    } else {
        return std::nullopt;
    }
}

FringeSearch::FringeSearch(Map& map, Node source, Node goal) : 
    map(map), source(source), goal(goal) {
}
