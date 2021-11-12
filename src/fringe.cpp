#include "fringe.h"

#include <limits.h>

Point neighbour_offsets[4] = {
    {1, 0},
    {0, 1},
    {-1, 0},
    {0, -1},
};

std::list<Node> FringeSearch::search(Map& map, Node source, Node goal) {
    cache.clear();
    cache.resize(map.width * map.height);

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
        Point a = map.node_to_point(i);
        Point b = map.node_to_point(goal);

        return distance(a, b);
    };

    int flimit = h(source);
    int found = false;
    FringeEntry nnode = fringe_list.end();

    int it = 0;
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
            for(int i = 0; i < 4; i++) {
                Point neigh = neighbour_offsets[i];
                neigh.x += np.x;
                neigh.y += np.y;

                if(neigh.x >= 0 && neigh.x < map.width && neigh.y >= 0 && neigh.y < map.height) {
                    Node s = map.point_to_node(neigh);
                    if(!map.data[s]) {
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
        Node it = *nnode;
        
        std::list<Node> shortest_path = { goal };
        for (Node v = cache[goal].parent; v != shortest_path.front(); v = cache[v].parent) {
            shortest_path.push_front(v);
        }

        return shortest_path;
    } else {
        return std::list<Node>();
    }
}