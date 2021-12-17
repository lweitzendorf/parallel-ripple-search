#include <iostream>
#include <raylib.h>

#include "benchmark/benchmarks.h"
#include "utility/Timer.h"
#include "ripple/HighLevelGraph.h"

void draw_walls(Image img, Map &map) {
  for (int y = 0; y < map.height(); y++) {
    for (int x = 0; x < map.width(); x++) {
      if (!map.get(Point(x, y))) {
        ImageDrawPixel(&img, x, y, BLACK);
      }
    }
  }
}

void draw_high_level_graph(GridHighLevelGraph& graph) {
  for(int y = 0; y < graph.grid.height(); y++) {
    for(int x = 0; x < graph.grid.width(); x++) {
      Point p(x, y);
      if(graph.grid.get(p)) {
          Point v = graph.grid_to_map(p);
          DrawCircleLines(v.x, v.y, 5.0f, BLUE);
          DrawPixel(v.x, v.y, BLUE);
      }
    }
  }

  for(int u = 0; u < graph.adj.size(); u++) {
    Point up = graph.grid.node_to_point(u);
    Point uu = graph.grid_to_map(up);

    for(auto v: graph.adj[u]) {
      Point vp = graph.grid.node_to_point(v);
      Point vv = graph.grid_to_map(vp);
      DrawLine(uu.x, uu.y, vv.x, vv.y, BLUE);
    }
  }
}

void draw_path(std::vector<Point> path, Color color, float size, bool edges) {
  for(auto p : path) {
    DrawRing({(float)p.x, (float)p.y}, size - 1, size + 1, 0, 360.0f, 16, color);
    DrawCircleLines(p.x, p.y, size, color);
    DrawPixel(p.x, p.y, color);
  }

  if(edges) {
    for(int i = 0; i < path.size() - 1; i++) {
      auto a = path[i];
      auto b = path[i + 1];
      DrawLine(a.x, a.y, b.x, b.y, color);
    }
  }
}

void draw_source_and_goal(Point source, Point goal) {
  //DrawCircleLines(source.x, source.y, 10.0f, GREEN);
  DrawRing({(float)source.x, (float)source.y}, 8.0f, 12.0f, 0, 360.0f, 16, GREEN);
  DrawPixel(source.x, source.y, GREEN);

  DrawRing({(float)goal.x, (float)goal.y}, 8.0f, 12.0f, 0, 360.0f, 16, RED);
  DrawPixel(goal.x, goal.y, RED);
}

int main(int argc, char **argv) {
  SetTraceLogLevel(LOG_NONE);

  // Load benchmark set
  std::string bench = "sc1";
  auto maps = load_maps("../benchmarks/" + bench + "-map");
  std::cout << "Benchmark: " << bench <<" - loaded " << maps.size() << " maps" << std::endl;

  
  // Initialize window
  const int SCREEN_WIDTH = maps[0].second.width();
  const int SCREEN_HEIGHT = maps[0].second.height();
  const Rectangle WINDOW = {0, 0, (float)SCREEN_WIDTH, (float)SCREEN_HEIGHT};

  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Graph View");
  SetTargetFPS(60);


  // Create all map images
  std::vector<Texture2D> textures;
  std::vector<std::vector<Scenario>> scenarios;

  uint64_t high_search_ns = 0;
  uint64_t total_scenarios_count = 0;
  for (auto &m : maps) {
    auto& map = m.second;
    auto scen = load_scenarios("../benchmarks/" + bench + "-scen/", m.first);
    

    GridHighLevelGraph graph(map);

    Timer t;
    t.start();
    for(int i = 0; i < scen.size(); i++) {
      auto s = scen[i];
      graph.create_high_level_path(map.point_to_node(s.source), map.point_to_node(s.goal), 4);
      total_scenarios_count++;
    }
    t.stop();
    high_search_ns += t.get_nanoseconds();
    
    
    Image img = GenImageColor(map.width(), map.height(), WHITE);
    draw_walls(img, map);
    textures.push_back(LoadTextureFromImage(img));

    scenarios.push_back(std::move(scen));

    UnloadImage(img);
  }

  printf("Total high search time: %.3fs (%.3fus avg) | %d scenarios\n", (double)high_search_ns / 1e9, 
    (high_search_ns / 1e3) / total_scenarios_count, (int)total_scenarios_count);


  // Main loop
  size_t scenario_index = 300;
  int num_non_essential_threads = 4;
  for (unsigned texture_index = 0;
       !WindowShouldClose();) // Detect window close button or ESC key
  {
    if (IsKeyPressed(KEY_SPACE)) {
        ++texture_index %= textures.size();
        SetWindowSize(maps[texture_index].second.width(), maps[texture_index].second.height());
        scenario_index = 300;

        printf("Selected map: %s\n", maps[texture_index].first.c_str());
    }  
    if(IsKeyPressed(KEY_P)) {
      scenario_index += 100;
    }
    if(IsKeyPressed(KEY_O)) {
      scenario_index -= 100;
    }
    if(IsKeyPressed(KEY_M)) {
      num_non_essential_threads += 1;
    }
    if(IsKeyPressed(KEY_N)) {
      scenario_index -= 1;
    }

    // Clamp user variables
    scenario_index = std::min(std::max(0ul, scenario_index), scenarios[texture_index].size() - 1);
    num_non_essential_threads = std::max(2, num_non_essential_threads);

    BeginDrawing();
    
    // Draw map
    Map& map = maps[texture_index].second;
    Rectangle rect = {0, 0, (float)map.width(), (float)map.height()};
    DrawTextureQuad(textures[texture_index], {1, 1}, {0, 0}, rect, WHITE);

    // Run high level path construction for scenario
    Scenario s = scenarios[texture_index][scenario_index];
    GridHighLevelGraph graph(map);
    auto path = graph.get_full_path(s.source, s.goal);
    auto starting_points = graph.refine_high_level_path(path, num_non_essential_threads);
    assert(starting_points.size() <= num_non_essential_threads);

    // Draw results
    //draw_high_level_graph(graph);
    draw_path(path, BLUE, 5.0, true);
    draw_path(starting_points, PINK, 10.0, false);
    draw_source_and_goal(s.source, s.goal);

    EndDrawing();
  }

  CloseWindow();
}
