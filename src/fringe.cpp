#include "fringe.h"

#include <limits.h>

#include <stdio.h>

std::list<Node> FringeSearch::search() {
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
            for(int i = 0; i < 4; i++) {
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
        // Node it = *nnode;
        
        std::list<Node> shortest_path = { goal };
        for (Node v = cache[goal].parent; v != shortest_path.front(); v = cache[v].parent) {
            shortest_path.push_front(v);
        }

        return shortest_path;
    } else {
        return std::list<Node>();
    }
}

FringeSearch::FringeSearch(Map& map) : 
    map(map) {
}

FringeSearchStep FringeSearch::step() {
    FringeSearchStep result;

    while(!fringe_list.empty()) {
        if(nnode == fringe_list.end()) {
            fmin = INT_MAX;
            nnode = fringe_list.begin();
        }

        do {
            if(*nnode == goal) {
                result.state = FringeSearchStepState::FOUND;
                return result;
            }

            auto& n = cache[*nnode];

            int g = n.g;
            int f = g + map.distance(*nnode, goal);

            if(f > flimit) {
                fmin = std::min(fmin, f);
                nnode++;
                continue;
            }

            
            Point np = map.node_to_point(*nnode);

            // For each neighbour
            for(int i = 0; i < 4; i++) {
                Point neigh = Map::neighbour_offsets[i];
                neigh.x += np.x;
                neigh.y += np.y;

                if(map.in_bounds(neigh)) {
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
            
            // Update flimit before returning
            if(nnode == fringe_list.end()) {
                flimit = fmin;
            }

            result.state = FringeSearchStepState::OK;
            return result;

        } while(nnode != fringe_list.end());

        flimit = fmin;
    }
    
    result.state = FringeSearchStepState::UNREACHABLE;
    return result;
}

std::list<Node> FringeSearch::finalize_path() {
    // Node it = *nnode;
    
    std::list<Node> shortest_path = { goal };
    for (Node v = cache[goal].parent; v != shortest_path.front(); v = cache[v].parent) {
        shortest_path.push_front(v);
    }

    return shortest_path;
}


void FringeSearch::init(Node source, Node goal) {
    this->source = source;
    this->goal = goal;

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

    this->flimit = map.distance(source, goal);
    this->fmin = INT_MAX;

    nnode = fringe_list.end();
}
