
#include <iostream>
#include <vector>

#include <raylib.h>

#include "Timer.h"
#include "findpath.cpp"

#include "map.h"
#include "fringe.h"
#include "ripple.h"
#include "WeightedGraph.h"
#include "FileParser.h"

#include "Astar.h"

void check_source_and_goal(Map& map, Node source, Node goal) {
    // bounds check on source and goal
    if(source < 0 || source >= map.data.size()) {
        std::cout << "Source node out of bounds" << std::endl;
        exit(1);
    }

    if(goal < 0 || goal >= map.data.size()) {
        std::cout << "Goal node out of bounds" << std::endl;
        exit(1);
    }

    // check that both goal and source are not walls
    if(!map.data[source]) {
        std::cout << "Source is a wall" << std::endl;
        exit(1);
    }

    if(!map.data[goal]) {
        std::cout << "Goal is a wall" << std::endl;
        exit(1);
    }
}


void build_graph(WeightedGraph& g, Map& map) {
  int x_size = map.width;
  int y_size = map.height;

  std::vector<bool> is_path(x_size);
  for (int y = 0; y < y_size; y++) {
    for (int x = 0; x < x_size; x++) {
        bool current_is_path = map.get(x, y);

        vertex_t vertex_number = g.add_vertex(x, y);

        if (current_is_path) {
            if (x > 0 && is_path.at(x - 1))
            g.add_edge(vertex_number - 1, vertex_number, 1);

            if (y > 0 && is_path.at(x))
            g.add_edge(vertex_number - x_size, vertex_number, 1);
        }
        is_path.at(x) = current_is_path;
    }
  }
}

void draw_walls(Image img, Map& map) {
    for(int y = 0; y < map.height; y++) {
        for(int x = 0; x < map.width; x++) {
            if(!map.get(x, y)) {
                ImageDrawPixel(&img, x, y, BLACK);
            }
        }  
    }
}

Image test_boost_a_star(Map& map, Node source, Node goal) {
    WeightedGraph graph;
    build_graph(graph, map);
    Timer t;
    t.start();
    auto shortest_path = graph.a_star_search(source, goal);
    t.stop();
    printf("Boost A* search time: %.3fms\n", t.get_microseconds() / 1000.0);

    // Create image and draw walls
    Image img = GenImageColor(map.width, map.height, WHITE);
    draw_walls(img, map);

    for(auto it: shortest_path) {
        Point p = map.node_to_point(it);
        ImageDrawPixel(&img, p.x, p.y, RED);       
    }

    // Draw source and goal
    Point sp = map.node_to_point(source);
    Point gp = map.node_to_point(goal);
    ImageDrawPixel(&img, sp.x, sp.y, GREEN);
    ImageDrawPixel(&img, gp.x, gp.y, YELLOW);

    return img;
}

Image test_fringe_search(Map& map, Node source, Node goal) {
    // Run fringe search
    Timer t;
    t.start();

    
    FringeSearch fringe(map);
    fringe.init(source, goal);
    #if 0
    auto shortest_path = fringe.search();
    #else
    FringeSearchStep step;
    do {
        step = fringe.step();
    } while(step.state == FringeSearchStepState::OK);

    std::list<Node> shortest_path;
    if(step.state == FringeSearchStepState::FOUND) {
        shortest_path = fringe.finalize_path();
    }
    #endif

    t.stop();
    printf("Fringe search time: %.3fms\n", t.get_microseconds() / 1000.0);

    // Create image and draw walls
    Image img = GenImageColor(map.width, map.height, WHITE);
    draw_walls(img, map);

    // Draw visited nodes
    for(int y = 0; y < map.height; y++) {
        for(int x = 0; x < map.width; x++) {
            int index = y * map.width + x;
            if(fringe.cache[index].visited) {
                ImageDrawPixel(&img, x, y, LIME);
            }
        }
    }

    // Draw path
    for(auto it: shortest_path) {
        Point p = map.node_to_point(it);
        ImageDrawPixel(&img, p.x, p.y, RED);       
    }

    // Draw source and goal
    Point sp = map.node_to_point(source);
    Point gp = map.node_to_point(goal);
    ImageDrawPixel(&img, sp.x, sp.y, GREEN);
    ImageDrawPixel(&img, gp.x, gp.y, YELLOW);

    return img;
}

Image test_Astar(Map& map, Node source, Node goal) {
    Timer t;
    t.start();
    
    std::vector<Node> came_from (map.width * map.height);
    std::vector<double> cost_so_far (map.width * map.height, -1);
    a_star_search(map, source, goal, came_from, cost_so_far);
    std::vector<Node> path = reconstruct_path(source, goal, came_from);
    

    t.stop();
    printf("Astar search time: %.3fms\n", t.get_microseconds() / 1000.0);

    // Create image and draw walls
    Image img = GenImageColor(map.width, map.height, WHITE);
    draw_walls(img, map);


    // Draw path
    for(auto it: path) {
        Point p = map.node_to_point(it);
        ImageDrawPixel(&img, p.x, p.y, RED);       
    }

    // Draw source and goal
    Point sp = map.node_to_point(source);
    Point gp = map.node_to_point(goal);
    ImageDrawPixel(&img, sp.x, sp.y, GREEN);
    ImageDrawPixel(&img, gp.x, gp.y, YELLOW);

    return img;
}

Image test_Astar_2(Map& map, Node source, Node goal) {
    Timer t;
    t.start();
    
    //auto path = search(map, source, goal);
    std::list<Node> path; //placeholder

    t.stop();
    printf("Astar 2 search time: %.3fms\n", t.get_microseconds() / 1000.0);

    // Create image and draw walls
    Image img = GenImageColor(map.width, map.height, WHITE);
    draw_walls(img, map);


    // Draw path
    for(auto it: path) {
        Point p = map.node_to_point(it);
        ImageDrawPixel(&img, p.x, p.y, RED);       
    }

    // Draw source and goal
    Point sp = map.node_to_point(source);
    Point gp = map.node_to_point(goal);
    ImageDrawPixel(&img, sp.x, sp.y, GREEN);
    ImageDrawPixel(&img, gp.x, gp.y, YELLOW);

    return img;
}


Image test_ripple(Map& map, Node source, Node goal) {
    Timer t;
    t.start();
    
    RippleSearch ripple(map);
    ripple.search(source, goal);

    std::list<Node> path; //placeholder

    t.stop();
    printf("Ripple search time: %.3fms\n", t.get_microseconds() / 1000.0);

    // Create image and draw walls
    Image img = GenImageColor(map.width, map.height, WHITE);
    draw_walls(img, map);

    
    Color colors[NUM_THREADS] = {
        YELLOW,
        DARKGREEN,
        DARKPURPLE,
        MAGENTA,
        //SKYBLUE,
        //MAROON,
        //ORANGE,
        //LIME
    }; 

    // Draw cache
    std::atomic_thread_fence(std::memory_order_seq_cst);

    for(int y = 0; y < map.height; y++) {
        for(int x = 0; x < map.width; x++) {
            int index = y * map.width + x;
            auto id = ripple.cache[index].thread.load(std::memory_order_relaxed);
            if(id != THREAD_NONE) {
                ImageDrawPixel(&img, x, y, colors[id]);
            }
        }
    }

    // Draw path
    for(auto it: path) {
        Point p = map.node_to_point(it);
        ImageDrawPixel(&img, p.x, p.y, RED);       
    }

    // Draw source and goal
    Point sp = map.node_to_point(source);
    Point gp = map.node_to_point(goal);
    ImageDrawPixel(&img, sp.x, sp.y, GREEN);
    ImageDrawPixel(&img, gp.x, gp.y, YELLOW);

    return img;
}


int main(int argc, char** argv)
{
    //Disable raylib log
    SetTraceLogLevel(LOG_NONE);

    if(argc < 4) {
        std::cout << "Usage: " << argv[0] << " PATH START GOAL" << std::endl;
        exit(1);
    }

    Map map;

    std::string file_path(argv[1]);

    if (file_path.ends_with(".png")) {
      map.load_from_image_file(argv[1]);
    } else if (file_path.ends_with(".dot"))  {
      WeightedGraph graph;
      DotParser parser(argv[1]);
      parser.build_graph(graph);
      map = graph.create_map();
    } else {
      std::cout << "Unsupported file type!" << std::endl;
    }

    Node source = strtol(argv[2], nullptr, 10);
    Node goal = strtol(argv[3], nullptr, 10);

    check_source_and_goal(map, source, goal);

    // Run Boost A*
    Image boost_img = test_boost_a_star(map, source, goal);

    // Run fringe search
    Image fringe_img = test_fringe_search(map, source, goal);

    // Run our a star
    //Image Astar_img = test_Astar(map, source, goal);

    // Run reference a star
    //Image Astar2_img = test_Astar_2(map, source, goal);

    // Run ripple
    Image ripple_img = test_ripple(map, source, goal);

    // Initialization
    const int screen_width = 1000;
    const int screen_height = 1000;

    InitWindow(screen_width, screen_height, "Graph View");
    SetTargetFPS(60);

    Texture2D textures[] = {
        LoadTextureFromImage(ripple_img),
        LoadTextureFromImage(boost_img),
        LoadTextureFromImage(fringe_img),
        //LoadTextureFromImage(Astar_img),
        //LoadTextureFromImage(Astar2_img),
    };
    int texture_index = 0;
    int texture_count = sizeof(textures) / sizeof(textures[0]);

    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        if(IsKeyPressed(KEY_SPACE)) {
            texture_index = (texture_index + 1) % texture_count;
        }

        // Draw
        BeginDrawing();

        ClearBackground(RAYWHITE);

        Rectangle screen_rect = { 0, 0, screen_width, screen_height};
        DrawTextureQuad(textures[texture_index], {1, 1}, {0, 0}, screen_rect, WHITE);

        #if 0
        for(int i = 0; i < samples; i++) {
            auto p = nodes[i];
            DrawCircle(p.first * scale, p.second * scale, 5.0f, BLUE);

            if(i < samples - 1) {
                auto op = nodes[i + 1];
                DrawLine(p.first * scale, p.second * scale, op.first * scale, op.second * scale, BLUE);
            }
        }
        #endif

        EndDrawing();
    }

    // De-Initialization
    CloseWindow();        // Close window and OpenGL context

    return 0;
}