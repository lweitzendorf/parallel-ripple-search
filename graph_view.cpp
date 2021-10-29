
#include <fstream>
#include <vector>

#include "raylib.h"

int main(int argc, char** argv)
{
    const char* file_name = argv[1];
    std::ifstream graph_file(file_name);

    int x_size, y_size;
    graph_file >> x_size >> y_size;

    std::vector<char> map(x_size * y_size);
    for(int y = 0; y < y_size; y++) {
        for(int x = 0; x < x_size; x++) {
            char c; graph_file >> c;
            map[y * x_size + x] = c == '1';
        }  
    }

    int x_samples = x_size / 50; 
    int y_samples = y_size / 50;
    int samples = x_samples * y_samples;
    
    std::vector<std::pair<int, int>> nodes(samples);
    for(int i = 0; i < samples; i++) {
        int x = rand() % x_size;
        int y = rand() % y_size;

        nodes[i] = std::make_pair(x, y);
    }


 
    // Initialization
    const int scale = 2;
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
        for(int i = 0; i < samples; i++) {
            auto p = nodes[i];
            DrawCircle(p.first * scale, p.second * scale, 5.0f, BLUE);

            if(i < samples - 1) {
                auto op = nodes[i + 1];
                DrawLine(p.first * scale, p.second * scale, op.first * scale, op.second * scale, BLUE);
            }
        }

        EndDrawing();
    }

    // De-Initialization
    CloseWindow();        // Close window and OpenGL context

    return 0;
}

