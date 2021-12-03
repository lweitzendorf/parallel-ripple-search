#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>

#include <raylib.h>

#include "Astar.h"
#include "high_level_path.h"
#include "map.h"

#define K 3
#define N_POINTS 150

class HighLevelGraph {
  Map &map;

public:
  std::vector<std::vector<int>> knn_adj;
  std::vector<Point> nodes;

  HighLevelGraph(Map &map) : map(map) {}

  size_t size() { return nodes.size(); }

  std::vector<int> &neighbours(int i) { return knn_adj[i]; }

  double cost(int i, int j) { return Map::distance(nodes[i], nodes[j]); }

  int distance(int i, int j) { return Map::distance(nodes[i], nodes[j]); }
};

Path<Node> create_high_level_path(Map &map, Node source, Node goal) {
  Point source_p = map.node_to_point(source);
  Point goal_p = map.node_to_point(goal);

  // change rng
  std::random_device rd;
  //std::mt19937 gen(rd());
  std::mt19937 gen(12);
  std::uniform_int_distribution<> distrib_x(0, map.width() - 1);
  std::uniform_int_distribution<> distrib_y(0, map.height() - 1);
  int size = map.size();

  HighLevelGraph high(map);
  high.nodes.push_back(source_p);
  high.knn_adj.push_back(std::vector<int>());
  int iter = 1;

  // TODO: stop if a maximum number of points have been tried
  while (iter < N_POINTS + 1) {
    int x = distrib_x(gen);
    int y = distrib_y(gen);
    if (map.get(Point(x, y))) {
      std::multimap<int, int> distances;
      bool already_sampled = false;
      for (int i = 1; i < high.nodes.size(); i++) {
        int diff_x = high.nodes[i].x - x;
        int diff_y = high.nodes[i].y - y;
        int distance = diff_x * diff_x + diff_y * diff_y;
        if (distance == 0) {
          already_sampled = true;
          break;
        }
        distances.insert(std::make_pair(distance, i));
      }
      if (already_sampled)
        continue;
      int k = 0;
      std::vector<int> adj;
      for (auto it = distances.begin(); it != distances.end(); it++) {
        adj.push_back(it->second);
        high.knn_adj[it->second].push_back(iter);
        k++;
        if (k == K)
          break;
      }
      high.knn_adj.push_back(adj);
      high.nodes.push_back(Point(x, y));
      iter++;
    }
  }

  // add source and goal
  std::multimap<int, int> distances_source;
  for (int i = 1; i < high.nodes.size(); i++) {
    int diff_x = high.nodes[i].x - source_p.x;
    int diff_y = high.nodes[i].y - source_p.y;
    int distance = diff_x * diff_x + diff_y * diff_y;
    distances_source.insert(std::make_pair(distance, i));
  }
  int k_source = 0;
  std::vector<int> adj_source;
  for (auto it = distances_source.begin(); it != distances_source.end(); it++) {
    adj_source.push_back(it->second);
    high.knn_adj[it->second].push_back(0);
    k_source++;
    if (k_source == K)
      break;
  }
  high.knn_adj[0] = adj_source;

  std::multimap<int, int> distances_goal;
  for (int i = 0; i < high.nodes.size(); i++) {
    int diff_x = high.nodes[i].x - goal_p.x;
    int diff_y = high.nodes[i].y - goal_p.y;
    int distance = diff_x * diff_x + diff_y * diff_y;
    distances_goal.insert(std::make_pair(distance, i));
  }
  int k_goal = 0;
  std::vector<int> adj_goal;
  for (auto it = distances_goal.begin(); it != distances_goal.end(); it++) {
    adj_goal.push_back(it->second);
    high.knn_adj[it->second].push_back(high.nodes.size());
    k_goal++;
    if (k_goal == K)
      break;
  }
  high.knn_adj.push_back(adj_goal);

  high.nodes.push_back(goal_p);

  // Find high level path
  Path<Node> came_from =
      a_star_search_gen(high, 0, (int)high.nodes.size() - 1).value();
  Path<Node> path =
      reconstruct_path_gen(0, (int)high.nodes.size() - 1, came_from);
  for (auto &n : path) {
    n = map.point_to_node(high.nodes[n]);
  }

  return path;
}

#if 0

int main(int argc, char const *argv[]) {
  if (argc < 4) {
    std::cout << "Usage: " << argv[0] << " PATH START GOAL" << std::endl;
    exit(1);
  }

  // Disable raylib log
  SetTraceLogLevel(LOG_NONE);

  Map map;
  map.load_from_image_file(argv[1]);
  Node source = strtol(argv[2], nullptr, 10);
  Node goal = strtol(argv[3], nullptr, 10);

  Point source_p = map.node_to_point(source);
  Point goal_p = map.node_to_point(goal);

  // change rng
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distrib_x(0, map.width - 1);
  std::uniform_int_distribution<> distrib_y(0, map.height - 1);
  int size = map.height * map.width;

  std::vector<Point> high_nodes;
  std::vector<std::vector<int>> knn_adj;

  high_nodes.push_back(source_p);
  knn_adj.push_back(std::vector<int>());
  int iter = 1;
  while (iter < N_POINTS + 1) { // add early stop
    int x = distrib_x(gen);
    int y = distrib_y(gen);
    if (map.get(x, y)) {
      std::multimap<int, int> distances;
      bool already_sampled = false;
      for (int i = 1; i < high_nodes.size(); i++) {
        int diff_x = high_nodes[i].x - x;
        int diff_y = high_nodes[i].y - y;
        int distance = diff_x * diff_x + diff_y * diff_y;
        if (distance == 0) {
          already_sampled = true;
          break;
        }
        distances.insert(std::make_pair(distance, i));
      }
      if (already_sampled)
        continue;
      int k = 0;
      std::vector<int> adj;
      for (auto it = distances.begin(); it != distances.end(); it++) {
        adj.push_back(it->second);
        knn_adj[it->second].push_back(iter);
        k++;
        if (k == K)
          break;
      }
      knn_adj.push_back(adj);
      high_nodes.push_back(Point(x, y));
      iter++;
    }
  }

  // add source and goal
  std::multimap<int, int> distances_source;
  for (int i = 1; i < high_nodes.size(); i++) {
    int diff_x = high_nodes[i].x - source_p.x;
    int diff_y = high_nodes[i].y - source_p.y;
    int distance = diff_x * diff_x + diff_y * diff_y;
    distances_source.insert(std::make_pair(distance, i));
  }
  int k_source = 0;
  std::vector<int> adj_source;
  for (auto it = distances_source.begin(); it != distances_source.end(); it++) {
    adj_source.push_back(it->second);
    knn_adj[it->second].push_back(0);
    k_source++;
    if (k_source == K)
      break;
  }
  knn_adj[0] = adj_source;

  std::multimap<int, int> distances_goal;
  for (int i = 0; i < high_nodes.size(); i++) {
    int diff_x = high_nodes[i].x - goal_p.x;
    int diff_y = high_nodes[i].y - goal_p.y;
    int distance = diff_x * diff_x + diff_y * diff_y;
    distances_goal.insert(std::make_pair(distance, i));
  }
  int k_goal = 0;
  std::vector<int> adj_goal;
  for (auto it = distances_goal.begin(); it != distances_goal.end(); it++) {
    adj_goal.push_back(it->second);
    knn_adj[it->second].push_back(high_nodes.size());
    k_goal++;
    if (k_goal == K)
      break;
  }
  knn_adj.push_back(adj_goal);

  high_nodes.push_back(goal_p);

  std::cout << "graph:" << std::endl;
  for (int i = 0; i < high_nodes.size(); i++) {
    std::cout << i << std::endl;
    for (int j = 0; j < knn_adj[i].size(); j++) {
      std::cout << "\t" << knn_adj[i][j] << std::endl;
    }
  }
  std::cout << std::endl;
  // ASTAR
  Path<Node> came_from(high_nodes.size());
  std::vector<double> cost_so_far(high_nodes.size(), -1);
  a_star_search(high_nodes, knn_adj, 0, high_nodes.size() - 1, came_from,
                cost_so_far);
  for (auto a : came_from) {
    std::cout << a << " ";
  }
  std::cout << std::endl;
  Path<Node> path = reconstruct_path(0, high_nodes.size() - 1, came_from);
  for (auto a : path) {
    std::cout << a << std::endl;
  }

  const int screen_width = 1000;
  const int screen_height = 1000;

  InitWindow(screen_width, screen_height, "Graph View");
  SetTargetFPS(60);

  Image img = GenImageColor(map.width, map.height, BLACK);
  for (int y = 0; y < map.height; y++) {
    for (int x = 0; x < map.width; x++) {
      if (map.get(x, y)) {
        ImageDrawPixel(&img, x, y, WHITE);
      }
    }
  }

  for (int i = 1; i < high_nodes.size() - 1; i++) {
    auto p = high_nodes[i];
    ImageDrawCircle(&img, p.x, p.y, 5.0f, BLUE);
    char buff[30];
    snprintf(buff, sizeof(buff), "%d", i);
    ImageDrawText(&img, buff, p.x + 10, p.y + 10, 30, BLACK);

    for (int j = 0; j < knn_adj[i].size(); j++) {
      auto adj = high_nodes[knn_adj[i][j]];

      ImageDrawLine(&img, p.x, p.y, adj.x, adj.y, BLUE);
    }
  }

  auto high_source = high_nodes[0];
  ImageDrawCircle(&img, high_source.x, high_source.y, 50.0f, GREEN);

  for (int j = 0; j < knn_adj[0].size(); j++) {
    auto adj = high_nodes[knn_adj[0][j]];
    ImageDrawLine(&img, high_source.x, high_source.y, adj.x, adj.y, BLUE);
  }

  auto high_goal = high_nodes[high_nodes.size() - 1];
  ImageDrawCircle(&img, high_goal.x, high_goal.y, 50.0f, BLACK);

  for (int j = 0; j < knn_adj[high_nodes.size() - 1].size(); j++) {
    auto adj = high_nodes[knn_adj[high_nodes.size() - 1][j]];
    ImageDrawLine(&img, high_goal.x, high_goal.y, adj.x, adj.y, BLUE);
  }

  for (int i = 0; i < path.size(); i++) {
    auto p = high_nodes[path[i]];
    ImageDrawCircle(&img, p.x, p.y, 10.0f, RED);
    if (i < path.size() - 1) {
      auto p2 = high_nodes[path[i + 1]];
      ImageDrawLine(&img, p.x, p.y, p2.x, p2.y, RED);
    }
  }

  Texture2D texture = LoadTextureFromImage(img);

  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    // Update

    // Draw
    BeginDrawing();

    ClearBackground(RAYWHITE);

    Rectangle screen_rect = {0, 0, screen_width, screen_height};
    DrawTextureQuad(texture, {1, 1}, {0, 0}, screen_rect, WHITE);

    EndDrawing();
  }

  // De-Initialization
  CloseWindow(); // Close window and OpenGL context

  return 0;
}

#endif
