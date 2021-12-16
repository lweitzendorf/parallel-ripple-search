#include <iostream>
#include <optional>
#include <vector>

#include <raylib.h>

#include "utility/Timer.h"

#include "graph/WeightedGraph.h"
#include "reference/FringeSearch.h"
#include "reference/FringeSearchSimd.h"
#include "ripple/RippleSearch.h"
#include "utility/FileParser.h"

#include "reference/Astar.h"

#ifdef ONLY_EXPORT_IMGS
#define BOOST_IMG_FN "../imgs/boost_img.png"
#define FRINGE_IMG_FN "../imgs/fringe_img.png"
#define FRINGE_VEC_IMG_FN "../imgs/fringe_vec_img.png"
#define ASTAR_IMG_FN "../imgs/astar_img.png"
#define RIPPLE_IMG_FN "../imgs/ripple_img.png"
#endif

bool check_source_and_goal(Map &map, Node source, Node goal) {
  // bounds check on source and goal
  if (!map.in_bounds(map.node_to_point(source))) {
    std::cout << "Source node out of bounds" << std::endl;
    return false;
  }

  if (!map.in_bounds(map.node_to_point(goal))) {
    std::cout << "Goal node out of bounds" << std::endl;
    return false;
  }

  // check that both goal and source are not walls
  if (!map.get(map.node_to_point(source))) {
    std::cout << "Source is a wall" << std::endl;
    return false;
  }

  if (!map.get(map.node_to_point(goal))) {
    std::cout << "Goal is a wall" << std::endl;
    return false;
  }

  return true;
}

void build_graph(WeightedGraph &g, Map &map) {
  int x_size = map.width();
  int y_size = map.height();

  std::vector<bool> is_path(x_size);
  for (int y = 0; y < y_size; y++) {
    for (int x = 0; x < x_size; x++) {
      bool current_is_path = map.get(Point(x, y));

      vertex_t vertex_number = g.add_vertex(Point(x, y));

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

void draw_walls(Image img, Map &map) {
  for (int y = 0; y < map.height(); y++) {
    for (int x = 0; x < map.width(); x++) {
      if (!map.get(Point(x, y))) {
        ImageDrawPixel(&img, x, y, BLACK);
      }
    }
  }
}

template <typename Iterator> void print_path(Iterator begin, Iterator end) {
  int path_length = 0;

  while (begin != end) {
    std::cout << *begin++ << " ";
    path_length++;
  }

  if (path_length > 0) {
    std::cout << std::endl << "Path length: " << path_length;
  } else {
    std::cout << "No path found!";
  }
  std::cout << std::endl;
}

Image test_boost_a_star(Map &map, Node source, Node goal) {
  WeightedGraph graph;
  build_graph(graph, map);
  Timer t;
  t.start();
  auto path = graph.a_star_search(source, goal);
  t.stop();
  printf("Boost A* search time: %.3fms (%d nodes)\n",
         t.get_microseconds() / 1000.0, (int)path.size());
  // print_path(path.begin(), path.end());
  // std::cout << std::endl;

  // Create image and draw walls
  Image img = GenImageColor(map.width(), map.height(), WHITE);
  draw_walls(img, map);

  for (auto it : path) {
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

Image test_Astar(Map &map, Node source, Node goal) {
  Timer t;
  t.start();

  Path<Node> path = a_star_search(map, source, goal).value_or(Path<Node>());

  t.stop();
  printf("Astar search time: %.3fms (%d nodes)\n",
         t.get_microseconds() / 1000.0, (int)path.size());
  // print_path(path.begin(), path.end());
  // std::cout << std::endl;

  // Create image and draw walls
  Image img = GenImageColor(map.width(), map.height(), WHITE);
  draw_walls(img, map);

  // Draw path
  for (auto it : path) {
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

Image test_Astar_2(Map &map, Node source, Node goal) {
  Timer t;
  t.start();

  // auto path = search(map, source, goal);
  Path<Node> path; // placeholder

  t.stop();
  printf("Astar 2 search time: %.3fms\n", t.get_microseconds() / 1000.0);
  // print_path(path.begin(), path.end());
  // std::cout << std::endl;

  // Create image and draw walls
  Image img = GenImageColor(map.width(), map.height(), WHITE);
  draw_walls(img, map);

  // Draw path
  for (auto it : path) {
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

template <typename Search>
Image test_search(std::string name, Map &map, Node source, Node goal,
                  std::function<void(Image &, Map &, Search &)> draw) {
  Timer t;
  t.start();

  Search search(map, source, goal);
  auto path = search.search().value_or(Path<Node>());

  t.stop();

  printf("%s time: %.3fms (%d nodes)\n", name.c_str(),
         (double)t.get_milliseconds(), (int)path.size());

  //print_path(path.begin(), path.end());

  // Create image and draw walls
  Image img = GenImageColor(map.width(), map.height(), WHITE);

  // Custom draw
  draw(img, map, search);

  // Draw walls
  draw_walls(img, map);

  // Draw path
  for (auto it : path) {
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

template <typename Fringe>
void fringe_draw(Image &img, Map &map, Fringe &search) {
  for (int y = 0; y < map.height(); y++) {
    for (int x = 0; x < map.width(); x++) {
      int index = map.point_to_node(Point(x, y));
      if (search.cache[index].visited) {
        ImageDrawPixel(&img, x, y, LIME);
      }
    }
  }
};

void ripple_draw(Image &img, Map &map, RippleSearch &search) {
  Color colors[] = { YELLOW, DARKGREEN, SKYBLUE, DARKPURPLE, LIME, BROWN, VIOLET, YELLOW,
                     BLUE, ORANGE, PINK, BLUE, GREEN,  DARKBROWN, DARKBLUE, DARKGRAY,
                     BEIGE, GOLD, GRAY, MAGENTA,
                     CLITERAL(Color){ 125, 33, 55, 255 },
                     CLITERAL(Color){ 33, 125, 55, 255 },
                     CLITERAL(Color){ 55, 33, 125, 255 },
                     CLITERAL(Color){ 200, 100, 50, 255 }};

  // Draw cache
  std::atomic_thread_fence(std::memory_order_seq_cst);

  for (int y = 0; y < map.height(); y++) {
    for (int x = 0; x < map.width(); x++) {
      auto id = search.get_owner(Point(x, y));
      if (id != THREAD_NONE) {
        ImageDrawPixel(&img, x, y, colors[id]);
      }
    }
  }
}

int main(int argc, char **argv) {
  // Disable raylib log
  SetTraceLogLevel(LOG_NONE);

  if (argc < 4) {
    std::cout << "Usage: " << argv[0] << " PATH START GOAL" << std::endl;
    return 1;
  }

  Map map;
  std::string file_path(argv[1]);

  if (file_path.ends_with(".png")) {
    map.load_from_image_file(argv[1]);
  } else if (file_path.ends_with(".dot")) {
    WeightedGraph graph;
    DotParser(argv[1]).build_graph(graph);
    map = graph.create_map();
  } else {
    std::cout << "Unsupported file type!" << std::endl;
    return 2;
  }

  Node source = strtol(argv[2], nullptr, 10);
  Node goal = strtol(argv[3], nullptr, 10);

  if (!check_source_and_goal(map, source, goal))
    return 3;

  const std::vector<Image> images = {
      test_search<RippleSearch>("Ripple", map, source, goal, ripple_draw),
      test_search<FringeSearchSimd>("Fringe Vec", map, source, goal,
                                    fringe_draw<FringeSearchSimd>),
      test_search<FringeSearch>("Fringe", map, source, goal,
                                fringe_draw<FringeSearch>),
      test_boost_a_star(map, source, goal),
      test_Astar(map, source, goal),
  };

#ifdef ONLY_EXPORT_IMGS
  bool ripple_written = ExportImage(images[0], RIPPLE_IMG_FN);
  bool fringe_vec_written = ExportImage(images[1], FRINGE_VEC_IMG_FN);
  bool fringe_written = ExportImage(images[2], FRINGE_IMG_FN);
  bool astar_written = ExportImage(images[3], ASTAR_IMG_FN);
  bool boost_written = ExportImage(images[4], BOOST_IMG_FN);
#else

  // Initialization
  const int SCREEN_WIDTH = 1000;
  const int SCREEN_HEIGHT = 1000;
  const Rectangle WINDOW = {0, 0, SCREEN_WIDTH, SCREEN_HEIGHT};

  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Graph View");
  SetTargetFPS(60);

  std::vector<Texture2D> textures;
  for (auto &img : images) {
    textures.push_back(LoadTextureFromImage(img));
  }

  for (unsigned texture_index = 0;
       !WindowShouldClose();) // Detect window close button or ESC key
  {
    if (IsKeyPressed(KEY_SPACE))
      ++texture_index %= textures.size();

    BeginDrawing();
    DrawTextureQuad(textures[texture_index], {1, 1}, {0, 0}, WINDOW, WHITE);
    EndDrawing();
  }

  CloseWindow();

#endif

  return 0;
}
