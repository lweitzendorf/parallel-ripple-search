#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <random>
#include <map>
#include <unordered_map>
#include "map.h"

#include "raylib.h"

#define K 5

int main(int argc, char const *argv[]) {
    if(argc < 4) {
        std::cout << "Usage: " << argv[0] << " PATH START GOAL" << std::endl;
        exit(1);
    }

    //Disable raylib log
    SetTraceLogLevel(LOG_NONE);

    Map map;
    map.load_from_image_file(argv[1]);
    Node source = strtol(argv[2], nullptr, 10);
    Node goal = strtol(argv[3], nullptr, 10);

    Point source_p = map.node_to_point(source);
    Point goal_p = map.node_to_point(goal);

    //change rng
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib_x(0, map.width-1);
    std::uniform_int_distribution<> distrib_y(0, map.height-1);
    int size = map.height* map.width;
    int n_points = 10;
    std::vector<Point> high_nodes;
    std::vector<std::vector<int>> knn_adj;
    while(n_points){
        int x = distrib_x(gen);
        int y = distrib_y(gen);
        if(map.get(x, y )){
            std::multimap<int, int> distances;
            bool already_sampled = false;
            for(int i = 0; i < high_nodes.size(); i++){
                int diff_x = high_nodes[i].x - x;
                int diff_y = high_nodes[i].y - y;
                int distance = diff_x*diff_x + diff_y*diff_y;
                if(distance == 0){
                    already_sampled = true; break;
                }
                distances.insert(std::make_pair(distance, i));
            }
            if(already_sampled)
                continue;
            int k = 0;
            std::vector<int> adj;
            for(auto it = distances.begin(); it != distances.end(); it++){
                adj.push_back(it->second);
                k++;
                if(k == K)
                    break;
            }
            knn_adj.push_back(adj);
            high_nodes.push_back(Point(x, y));
            n_points--;
        } 
        
    }

    //add source and goal
    std::multimap<int, int> distances_source;
    for(int i = 0; i < high_nodes.size(); i++){
        int diff_x = high_nodes[i].x - source_p.x;
        int diff_y = high_nodes[i].y - source_p.y;
        int distance = diff_x*diff_x + diff_y*diff_y;
        distances_source.insert(std::make_pair(distance, i));
    }
    int k_source = 0;
    std::vector<int> adj_source;
    for(auto it = distances_source.begin(); it != distances_source.end(); it++){
        adj_source.push_back(it->second);
        k_source++;
        if(k_source == K)
            break;
    }
    knn_adj.push_back(adj_source);

    std::multimap<int, int> distances_goal;
    for(int i = 0; i < high_nodes.size(); i++){
        int diff_x = high_nodes[i].x - goal_p.x;
        int diff_y = high_nodes[i].y - goal_p.y;
        int distance = diff_x*diff_x + diff_y*diff_y;
        distances_goal.insert(std::make_pair(distance, i));
    }
    int k_goal = 0;
    std::vector<int> adj_goal;
    for(auto it = distances_goal.begin(); it != distances_goal.end(); it++){
        adj_goal.push_back(it->second);
        k_goal++;
        if(k_goal == K)
            break;
    }
    knn_adj.push_back(adj_goal);

    high_nodes.push_back(source_p);
    high_nodes.push_back(goal_p);

    const int screen_width = 1000;
    const int screen_height = 1000;

    InitWindow(screen_width, screen_height, "Graph View");
    SetTargetFPS(60);

    Image img = GenImageColor(map.width, map.height, BLACK);
    for(int y = 0; y < map.height; y++) {
        for(int x = 0; x < map.width; x++) {
            if(map.get(x, y)) {   
                ImageDrawPixel(&img, x, y, WHITE);
            }
        }  
    }

    for(int i = 0; i < high_nodes.size() - 2; i++) {
        auto p = high_nodes[i];
        ImageDrawCircle(&img, p.x, p.y, 5.0f, BLUE);

        for(int j = 0; j < knn_adj[i].size(); j++){
            auto adj = high_nodes[knn_adj[i][j]];
            ImageDrawLine(&img, p.x, p.y, adj.x , adj.y, BLUE);
        }
    }


    auto high_source = high_nodes[high_nodes.size()-2];
    ImageDrawCircle(&img, high_source.x, high_source.y, 50.0f, GREEN);

    for(int j = 0; j < knn_adj[high_nodes.size()-2].size(); j++){
        auto adj = high_nodes[knn_adj[high_nodes.size()-2][j]];
        ImageDrawLine(&img, high_source.x, high_source.y, adj.x , adj.y, BLUE);
    }

    auto high_goal = high_nodes[high_nodes.size()-1];
    ImageDrawCircle(&img, high_goal.x, high_goal.y, 50.0f, BLACK);

    for(int j = 0; j < knn_adj[high_nodes.size()-1].size(); j++){
        auto adj = high_nodes[knn_adj[high_nodes.size()-1][j]];
        ImageDrawLine(&img, high_goal.x, high_goal.y, adj.x , adj.y, BLUE);
    }
    Texture2D texture = LoadTextureFromImage(img);

    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update

        // Draw
        BeginDrawing();

        ClearBackground(RAYWHITE);

        Rectangle screen_rect = { 0, 0, screen_width, screen_height};
        DrawTextureQuad(texture, {1, 1}, {0, 0}, screen_rect, WHITE);

        EndDrawing();
    }

    // De-Initialization
    CloseWindow();        // Close window and OpenGL context

    return 0;
}