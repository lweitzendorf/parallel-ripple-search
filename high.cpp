#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <random>
#include <map>
#include <unordered_map>

#include "raylib.h"

int main(int argc, char const *argv[]) {
    const char* file_name = argv[1];
    const int K = 5;
    std::ifstream graph_file(file_name);

    int x_size, y_size;
    graph_file >> x_size >> y_size;

    std::vector<char> map(x_size * y_size);
    for(int y = 0; y < y_size; y++){
        for(int x = 0; x < x_size; x++){
            char c; graph_file >> c;
            map[y*x_size + x] = c == '1';
        }
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib_x(0, x_size-1);
    std::uniform_int_distribution<> distrib_y(0, y_size-1);
    int size = y_size * x_size;
    int n_points = size / 100;
    std::vector<std::pair<int,int>> high_nodes;
    std::vector<std::vector<int>> knn_adj;
    while(n_points){
        int x = distrib_x(gen);
        int y = distrib_y(gen);
        if(map[y*x_size + x]){
            n_points--;
            std::multimap<int, int> distances;
            for(int i = 0; i < high_nodes.size(); i++){
                int diff_x = high_nodes[i].first - x;
                int diff_y = high_nodes[i].second - y;
                int distance = diff_x*diff_x + diff_y*diff_y;
                distances.insert(std::make_pair(distance, i));
            }
            int k = 0;
            std::vector<int> adj;
            for(auto it = distances.begin(); it != distances.end(); it++){
                adj.push_back(it->second);
                k++;
                if(k == K)
                    break;
            }
            knn_adj.push_back(adj);
            high_nodes.push_back(std::make_pair(x, y));
        }
        
    }

    const int scale = 4;
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

    Texture2D texture = LoadTextureFromImage(img);

    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update

        // Draw
        BeginDrawing();

        ClearBackground(RAYWHITE);

        DrawTextureEx(texture, {0, 0}, 0, scale, WHITE);
        for(int i = 0; i < high_nodes.size(); i++) {
            auto p = high_nodes[i];
            DrawCircle(p.first * scale, p.second * scale, 5.0f, BLUE);

            for(int j = 0; j < knn_adj[i].size(); j++){
                auto adj = high_nodes[knn_adj[i][j]];
                DrawLine(p.first * scale, p.second * scale, adj.first * scale, adj.second * scale, BLUE);
            }
        }

        EndDrawing();
    }

    // De-Initialization
    CloseWindow();        // Close window and OpenGL context

    return 0;
}