#include "FringeVecSearch.h"

#include <cmath>

#include <immintrin.h>


std::optional<std::vector<Node>> FringeVecSearch::search(Node source, Node goal) {
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

    Point goalp = map.node_to_point(goal);
    float flimit = map.distance(source, goal);
    bool found = false;

    int32_t current_list = 0;
    while(!found && !now_list.empty()) {
        float fmin = std::numeric_limits<float>::max();
        
        do {
            Node node = now_list.back();
            now_list.pop_back();

            if(node == goal) {
                found = true;
                break;
            }


            auto& n = cache[node];
            if (n.list_index != current_list) {
                continue;
            }

            n.list_index = -1;
            

            Point np = map.node_to_point(node);

            float g = n.g;
            float f = g + map.distance(np, goalp);

            if(f > flimit + 1) {
                fmin = std::min(fmin, f);
                later_list.push_back(node);
                n.list_index = 1 - current_list;
                continue;
            }

            
            
        #if AVX512_ENABLED
            __m256i zero = _mm256_setzero_si256();
            __m256i ones = _mm256_set1_epi32(-1);

            // node coordinates
            __m256i n_x = _mm256_set1_epi32(np.x);
            __m256i n_y = _mm256_set1_epi32(np.y);
            
            // Compute neighbour coordinates
            __m256i offset_x = _mm256_set_epi32(-1, -1,  1,  1, -1,  0,  1,  0);
            __m256i offset_y = _mm256_set_epi32(-1,  1, -1,  1,  0, -1,  0,  1);

            __m256i neigh_x = _mm256_add_epi32(n_x, offset_x);
            __m256i neigh_y = _mm256_add_epi32(n_y, offset_y);

            // Check bounds
            #if 1
            __mmask8 x_ge0 = _mm256_cmpge_epi32_mask(neigh_x, zero);
            __mmask8 y_ge0 = _mm256_cmpge_epi32_mask(neigh_y, zero);

            __m256i size_x = _mm256_set1_epi32(map.width());
            __mmask8 x_lt_size = _mm256_cmplt_epi32_mask(neigh_x, size_x);

            __m256i size_y = _mm256_set1_epi32(map.height());
            __mmask8 y_lt_size = _mm256_cmplt_epi32_mask(neigh_y, size_y);
            
            // inbounds = (x >= 0 && y >= 0) && (x < width, && y < height)
            __mmask8 in_bounds = _kand_mask8(_kand_mask8(x_ge0, y_ge0), _kand_mask8(x_lt_size, y_lt_size));
            #else
            __m256i size_x = _mm256_set1_epi32(map.width());
            #endif

            // Compute neighbour index
            // s = x + y * width
            __m256i s = _mm256_add_epi32(neigh_x, _mm256_mullo_epi32(neigh_y, size_x));
            __m256i s_scaled = _mm256_slli_epi32(s, 4);

            // Check walls for neighbours in bounds
            __m256i walls32  = _mm256_mmask_i32gather_epi32(zero, in_bounds, s, map.data_ptr(), 1);
            //__m256i walls32  = _mm256_i32gather_epi32((int*)map.data_ptr(), s, 1);
            __m128i walls = _mm256_cvtepi32_epi8(walls32);
            __mmask8 valid = _mm_cmpneq_epi8_mask(walls, _mm_setzero_si128());

            // Compute neighbour cost
            float cost2 = sqrtf(2);
            __m256 additional_cost = _mm256_set_ps(cost2, cost2, cost2,  cost2, 1,  1,  1,  1);
            __m256 cost = _mm256_add_ps(additional_cost, _mm256_set1_ps(g));

            // Gather visited
            __m256i visited_value = _mm256_mmask_i32gather_epi32(zero, valid, s_scaled, ((uint8_t*)cache.data()) + offsetof(FringeNodeVec, visited), 1);
            __mmask8 visited = _mm256_cmpneq_epi32_mask(visited_value, zero);

            // Gather cost for visited nodes
            __m256 prev_cost = _mm256_mmask_i32gather_ps(_mm256_setzero_ps(), visited, s_scaled, ((uint8_t*)cache.data()) + offsetof(FringeNodeVec, g), 1);

            // Check if new cost is cheaper
            __mmask8 cheaper = _mm256_cmp_ps_mask(cost, prev_cost, _CMP_LT_OQ);

            // Update only valid node that are cheaper or unvisited nodes (can probably use an andn here)
            __mmask8 update_mask = _kand_mask8(valid, _kor_mask8(cheaper, _knot_mask8(visited)));

            // Scatter visited, new cost, and parent for nodes to update
            _mm256_mask_i32scatter_epi32(((uint8_t*)cache.data()) + offsetof(FringeNodeVec, visited), update_mask, s_scaled, ones, 1);
            _mm256_mask_i32scatter_ps   (((uint8_t*)cache.data()) + offsetof(FringeNodeVec, g      ), update_mask, s_scaled, cost, 1);
            _mm256_mask_i32scatter_epi32(((uint8_t*)cache.data()) + offsetof(FringeNodeVec, parent ), update_mask, s_scaled, _mm256_set1_epi32(node), 1);

            // Gather list index
            __m256i curr_list = _mm256_set1_epi32(current_list);
            __m256i list_index = _mm256_mmask_i32gather_epi32(curr_list, update_mask, s_scaled, ((uint8_t*)cache.data()) + offsetof(FringeNodeVec, list_index), 1);

            // Check if not in current list
            __mmask8 to_push = _mm256_cmpneq_epi32_mask(curr_list, list_index);
            
            // Store current list
            _mm256_mask_i32scatter_epi32(((uint8_t*)cache.data()) + offsetof(FringeNodeVec, list_index), to_push, s_scaled, curr_list, 1);

            // Compress and push into current list
            uint32_t mask_size = __builtin_popcount(_cvtmask8_u32(to_push)); 
            now_list.resize(now_list.size() + mask_size);
            Node* now_end = now_list.data() + now_list.size() - mask_size;
            _mm256_mask_compressstoreu_epi32(now_end, to_push, s);
        #else 
            // For each neighbour
            for(int i = 0; i < Map::NEIGHBOURS_COUNT; i++) {
                Point neigh = Map::neighbour_offsets[i];
                neigh.x += np.x;
                neigh.y += np.y;

                
                if (map.in_bounds(neigh)) 
                {

                    // Avoid cutting corners
                    #if 0
                    if(i < 4) {
                        Point n1(neigh.x, np.y);
                        Point n2(np.x, neigh.y);
                        if(!map.get(n1) || !map.get(n2)) {
                            continue;
                        }
                    }
                    #endif

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
        #endif

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

FringeVecSearch::FringeVecSearch(Map& map) :
    map(map) {
}