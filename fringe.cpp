#include <stdlib.h>
#include <limits.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <list>
#include <iostream>
#include <fstream>

#include <raylib.h>

typedef int Node;
#define INVALID_NODE -1

typedef std::list<Node> FringeList;
typedef FringeList::iterator FringeEntry;

struct FringeNode {
    bool visited = false;
    bool in_list;
    int g; // Cost from source to node
    Node parent;
    FringeEntry list_entry;
};

struct Point {
    int x;
    int y;
};

Point neighbour_offsets[4] = {
    {1, 0},
    {0, 1},
    {-1, 0},
    {0, -1},
};

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cout << "Usage: " << argv[0] << " <graph_file> <source_node> <target_node>" << std::endl;
        return -1;
    }

    // Parse file, source node and goal node
    const char* file_name = argv[1];
    const int K = 5;
    std::ifstream graph_file(file_name);

    int graph_width, graph_height;
    graph_file >> graph_width >> graph_height;

    std::vector<char> map(graph_width * graph_height);
    for(int y = 0; y < graph_height; y++){
        for(int x = 0; x < graph_width; x++){
            char c; graph_file >> c;
            map[y*graph_width + x] = c == '1';
        }
    }

    Node source = strtol(argv[2], nullptr, 10);
    Node goal = strtol(argv[3], nullptr, 10);

    // bounds check on source and goal
    if(source < 0 || source >= map.size()) {
        std::cout << "Source node out of bounds" << std::endl;
        exit(1);
    }

    if(goal < 0 || goal >= map.size()) {
        std::cout << "Goal node out of bounds" << std::endl;
        exit(1);
    }

    // check that both goal and source are not walls
    if(!map[source]) {
        std::cout << "Source is a wall" << std::endl;
        exit(1);
    }

    if(!map[goal]) {
        std::cout << "Goal is a wall" << std::endl;
        exit(1);
    }


    // Initialize cache and fringe list
    std::vector<FringeNode> cache(graph_width * graph_height);
    FringeList fringe_list;

    // Add source node to fringe list
    fringe_list.push_front(source);
    cache[source].in_list = true;
    cache[source].g = 0;
    cache[source].parent = source;
    cache[source].list_entry = fringe_list.begin();
    cache[source].visited = true;

    auto node_to_point = [&](Node i) {
        Point p;
        p.x = i % graph_width;
        p.y = i / graph_width;

        return p;
    };

    auto h = [&](Node i) {
        Point a = node_to_point(i);
        Point b = node_to_point(goal);

        return abs(b.x - a.x) + abs(b.y - a.y);
    };

    auto point_to_node = [&] (Point i){
        return i.y * graph_height + i.x;
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
                //std::cout << "f > flimit" << std::endl;
                fmin = std::min(fmin, f);
                nnode++;
                continue;
            }

            
            Point np = node_to_point(*nnode);

            // For each neighbour
            for(int i = 0; i < 4; i++) {
                Point neigh = neighbour_offsets[i];
                neigh.x += np.x;
                neigh.y += np.y;

                if(neigh.x >= 0 && neigh.x < graph_width && neigh.y >= 0 && neigh.y < graph_height) {
                    Node s = point_to_node(neigh);
                    if(!map[s]) {
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

    std::list<Node> shortest_path = { goal };
    if(nnode != fringe_list.end()) {
        Node it = *nnode;
        
        for (Node v = cache[goal].parent; v != shortest_path.front(); v = cache[v].parent) {
            shortest_path.push_front(v);
        }

        std::cout << "Total cost: " << shortest_path.size() << std::endl;
    } else {
        std::cout << "path not found" << std::endl;
    }


    const int scale = 3;
    const int x_size = graph_width;
    const int y_size = graph_height;
    const int screenWidth = scale * x_size;
    const int screenHeight = scale * y_size;

    InitWindow(screenWidth, screenHeight, "Graph View");
    SetTargetFPS(60);

    Image img = GenImageColor(x_size, y_size, BLACK);
    for(int y = 0; y < y_size; y++) {
        for(int x = 0; x < x_size; x++) {
            if(map[y * x_size + x]) {   
                ImageDrawPixel(&img, x, y, WHITE);
            }
        }  
    }

    for(auto it: shortest_path) {
        Point p = node_to_point(it);
        ImageDrawPixel(&img, p.x, p.y, RED);       
    }

    {
        Point sp = node_to_point(source);
        Point gp = node_to_point(goal);
        ImageDrawPixel(&img, sp.x, sp.y, GREEN);
        ImageDrawPixel(&img, gp.x, gp.y, YELLOW);
    }

    Texture2D texture = LoadTextureFromImage(img);

    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update

        // Draw
        BeginDrawing();

        ClearBackground(RAYWHITE);


        DrawTextureEx(texture, {0, 0}, 0, scale, WHITE);

        EndDrawing();
    }

    // De-Initialization
    CloseWindow();        // Close window and OpenGL context

    return 0;
}