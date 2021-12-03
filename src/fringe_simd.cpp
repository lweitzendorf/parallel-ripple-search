#include "fringe_simd.h"

#include <limits.h>

#include <stdio.h>
#include <math.h>

std::optional<std::vector<Node>> FringeSearchSimd::search() {
    cache.clear();
    cache.resize(map.size(), {});

    now_list.clear();
    later_list.clear();

    // Add source node to fringe list
    now_list.push_back(source);
    cache[source].list_index = 0;
    cache[source].g = 0;
    cache[source].parent = source;
    cache[source].visited = true;
    cache[source].list_index = 0;

    // Lambda for computing heuristic
    auto h = [&](Node i) {
        return map.distance(i, goal);
    };

    float flimit = h(source);
    bool found = false;

    char current_list = 0;
    while(!found && !now_list.empty()) {
        float fmin = INT_MAX;
        
        do {
            Node node = now_list.back();
            now_list.pop_back();

            if(node == goal) {
                found = true;
                break;
            }


            auto& n = cache[node];
            if(n.list_index != current_list) {
                continue;
            }

            n.list_index = -1;

            float g = n.g;
            float f = g + h(node);

            if(f > flimit + 1) {
                fmin = std::min(fmin, f);
                later_list.push_back(node);
                n.list_index = 1 - current_list;
                continue;
            }

            
            Point np = map.node_to_point(node);

            // For each neighbour
            for(int i = 0; i < 8; i++) {
                Point neigh = Map::neighbour_offsets[i];
                neigh.x += np.x;
                neigh.y += np.y;

                
                if (map.in_bounds(neigh)) {
                    Node s = map.point_to_node(neigh);

                    if(!map.get(neigh)) {
                        continue;
                    }

                    float gs = i >= 4 ? g + 1 : g + sqrtf(2);
                    if(cache[s].visited) {
                        float gi = cache[s].g;
                        if(gs >= gi) {
                            continue;
                        }
                    }

                    // Update visited, cost and parent
                    cache[s].visited = true;
                    cache[s].g = gs;
                    cache[s].parent = node;
                    
                    if(cache[s].list_index != current_list) {
                        now_list.push_back(s);
                        cache[s].list_index = current_list;
                    }
                } 
            }

        } while(!now_list.empty());

        std::swap(later_list, now_list);
        current_list = 1 - current_list;

        flimit = fmin;
    }

    if(found) {
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

FringeSearchSimd::FringeSearchSimd(Map& map, Node source, Node goal) : 
    map(map), source(source), goal(goal) {
}