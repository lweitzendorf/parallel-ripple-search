#include <fstream>
#include <iostream>
#include <raylib.h>
#include <filesystem>

#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>

#include "graph/WeightedGraph.h"
#include "graph/Map.h"
#include "reference/FringeSearch.h"
#include "reference/FringeSearchSimd.h"
#include "ripple/RippleSearch.h"
#include "utility/FileParser.h"
#include "utility/Timer.h"
#include "reference/Astar.h"

using namespace glm;
std::vector<Point> get_high_level_path(Map& map, Point source, Point goal, int num) {
  vec2 s(source.x, source.y);
  vec2 g(goal.x, goal.y);

  vec2 d = g - s;
  float len = length(d);
  vec2 n = normalize(d);
  vec2 perp = vec2(-n.y, n.x); //(0 1 | -1 0)

  float t = len / (float)(num - 1);

  std::vector<Point> result;
  for(int i = 0; i < num; i++) {
    vec2 v = n * (t * i) + s;
    Point p(v.x, v.y);
    
    bool ok = true;
    if(!map.get(p)) {
      ok = false;
      for(int j = 0; j < 20; j++) {
        vec2 offset = perp * (t * 0.3f * j); 
        vec2 u = v + offset;
        Point left(u.x, u.y);
        if(map.in_bounds(left) && map.get(left)) {
          p = left;
          ok = true;
          break;
        }

        u = v - offset;
        Point right(u.x, u.y);
        if(map.in_bounds(right) && map.get(right)) {
          p = right;
          ok = true;
          break;
        }
      }
    }
    
    if(ok)
      result.emplace_back(p);
  }

  return result;
}

int fail_count = 0;
int succ_count = 0;

uint64_t total_ns = 0;

std::vector<Point> high_level_grid(Map& map, Point source, Point goal, int num) {
  Timer t;
  t.start();

  const int GRID_SIZE = 16;

  Map grid(GRID_SIZE, GRID_SIZE);
  vec2 cell = vec2((float)map.width() / (GRID_SIZE), (float)map.height() / (GRID_SIZE));
  vec2 half = cell * 0.5f;

  vec2 s(source.x, source.y);
  vec2 g(goal.x, goal.y);
  Point closest_source, closest_goal;
  float dist_source = FLT_MAX;
  float dist_goal = FLT_MAX;

  std::vector<Point> pp;
  for(int y = 0; y < GRID_SIZE; y++) {
    for(int x = 0; x < GRID_SIZE; x++) {
      vec2 v = cell * vec2(x, y) + half;

      Point p(v.x, v.y);
      if(map.get(p)) {
        grid.set(Point(x, y), 1);

        float ds = distance2(v, s);
        if(ds < dist_source) {
          dist_source = ds;
          closest_source = Point(x, y);
        }

        float dg = distance2(v, g);
        if(dg < dist_goal) {
          dist_goal = dg;
          closest_goal = Point(x, y);
        }

        pp.push_back(p);
      } else {
        grid.set(Point(x, y), 0);
      }
    }
  }

  //printf("%d, %d -> %d, %d\n", closest_source.x, closest_source.y, closest_goal.x, closest_goal.y);
#if 1
  auto grid_path_opt = a_star_search(grid, grid.point_to_node(closest_source), grid.point_to_node(closest_goal));

  if(grid_path_opt.has_value()) {
    std::vector<Point> points;
    for(auto n: grid_path_opt.value()) {
      Point point = grid.node_to_point(n);
      vec2 p(point.x, point.y);
      vec2 v = cell * p + half;

      points.emplace_back(v.x, v.y);
    }
    succ_count++;


    t.stop();
    total_ns += t.get_microseconds();

    return points;
  } else {
    fail_count++;
    
    t.stop();
    total_ns += t.get_microseconds();
    
    return pp;
  }
#endif

  //std::vector<Node> a_star_search(grid, s, g);
}

std::vector<std::pair<std::string, Map>> load_maps(const std::string& dir) {
  std::vector<std::pair<std::string, Map>> result;
  for (const auto& entry : std::filesystem::directory_iterator(dir))
  {
    auto path = entry.path();
    std::fstream f(path);


    std::string trash;
    std::string type;
    int width, height;
    f >> trash >> type >> trash >> height >> trash >> width >> trash;

    Map map(width, height);

    for(int y = 0; y < height; y++) {
      for(int x = 0; x < width; x++) {
        char c; f >> c;
        if(c == '@' || c == 'T') {
          map.set(Point(x, y), 0);
        } else if(c == '.') {
          map.set(Point(x, y), 1);
        } else {
          assert(false);
        }
      }
    }

    result.push_back(std::make_pair(path.filename(), std::move(map)));
  }

  return result;
}

struct Scenario {
  Point source;
  Point goal;
  float cost;
};

std::vector<Scenario> load_scenarios(const std::string& dir, const std::string& map) {
  auto path = dir + map + ".scen";

  std::fstream f(path);

  std::string line;
  int bucket, width, height;
  
  std::vector<Scenario> result;

  std::string version;
  std::getline(f, version);


  while(std::getline(f, line)) {
    Scenario scen;

    std::istringstream iss(line);
    std::string name;
    iss >> bucket >> name >> width >> height >> scen.source.x >> scen.source.y >> scen.goal.x >> scen.goal.y >> scen.cost;
    assert(name == map);
    //assert(width == 512);
    //assert(height == 512);

    result.push_back(scen);
  }

  return result;
}


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

void flush_cache() {
  const int size = 128 * 1024 * 1024;
  volatile uint8_t *volatile data = (uint8_t *)malloc(size);
  for (int i = 0; i < size; i++) {
    data[i] = 0xcc;
    volatile uint8_t v = data[i];
  }

  free((void *)data);
}

template <typename Search>
void test_search(std::string name, Map &map, Node source, Node goal) {
  const int COUNT = 20;
  double min_time = 10e10, max_time = 0, avg_time = 0;
  double avg_init = 0;

  for (int i = 0; i < COUNT; i++) {
    flush_cache();

    Timer t;
    t.start();
    Search fringe(map, source, goal);
    auto shortest_path = fringe.search().value_or(Path<Node>());
    t.stop();

    double ms = t.get_microseconds() / 1000.0;
    min_time = std::min(min_time, ms);
    max_time = std::max(max_time, ms);
    avg_time += ms * (1.0 / COUNT);
  }

  printf("%s: %3.2fms avg | %3.2fms min | %3.2fms max\n", name.c_str(),
         avg_time, min_time, max_time);
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

uint64_t total_search_ns = 0;

template <typename Search>
Image test_search(std::string name, Map &map, Node source, Node goal,
                  std::function<void(Image &, Map &, Search &)> draw) {
  Timer t;
  t.start();

  Search search(map, source, goal);
  auto path = search.search().value_or(Path<Node>());

  t.stop();

  uint64_t ns = (double)t.get_nanoseconds();
  printf("%s time: %.3fms (%d nodes)\n", name.c_str(), ns / 1e6, (int)path.size());
  total_search_ns += ns;

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

int main(int argc, char **argv) {
  SetTraceLogLevel(LOG_NONE);

  //std::string bench = "bg512";
  std::string bench = "sc1";
  auto maps = load_maps("../benchmarks/" + bench + "-map");
  std::cout << "Benchmark: " << bench <<" - loaded " << maps.size() << " maps" << std::endl;

  const int SCREEN_WIDTH = maps[0].second.width();
  const int SCREEN_HEIGHT = maps[0].second.height();
  const Rectangle WINDOW = {0, 0, (float)SCREEN_WIDTH, (float)SCREEN_HEIGHT};

  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Graph View");
  SetTargetFPS(60);

  std::vector<Texture2D> textures;

  std::vector<std::vector<Scenario>> scenarios;

  for (auto &m : maps) {
    auto& map = m.second;
    auto scen = load_scenarios("../benchmarks/" + bench + "-scen/", m.first);
    
    //if(m.first == "AR0011SR.map") {
    {
      for(int i = 0; i < scen.size(); i++) {
        auto s = scen[i];
        high_level_grid(map, s.source, s.goal, 8);
        //printf("Scenario: (%d, %d) -> (%d, %d) cost: %.3f\n", s.source.x, s.source.y, s.goal.x, s.goal.y, s.cost);
      }
    }

    
    Image img = test_search<FringeSearchSimd>("Fringe Vec", map, map.point_to_node(scen.back().source), map.point_to_node(scen.back().goal),
                                    fringe_draw<FringeSearchSimd>);

    //Image img = GenImageColor(map.width(), map.height(), WHITE);
    //draw_walls(img, map);
    textures.push_back(LoadTextureFromImage(img));

    scenarios.push_back(std::move(scen));

    UnloadImage(img);
  }

  printf("total time: %.3fs | %d / %d (%.2f%%) failed\n", (double)total_search_ns / 1e9, fail_count, succ_count + fail_count, (double)fail_count / (double)(succ_count + fail_count) * 100);

  size_t scenario_index = 300;
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

    scenario_index = std::min(std::max(0ul, scenario_index), scenarios[texture_index].size() - 1);

    BeginDrawing();
    
    Rectangle rect = {0, 0, (float)maps[texture_index].second.width(), (float)maps[texture_index].second.height()};
    DrawTextureQuad(textures[texture_index], {1, 1}, {0, 0}, rect, WHITE);

    Scenario s = scenarios[texture_index][scenario_index];

#if 0
    auto path = get_high_level_path(maps[texture_index].second, s.source, s.goal, 8);
    for(auto p : path) {
      DrawCircleLines(p.x, p.y, 5.0f, BLUE);
      DrawPixel(p.x, p.y, BLUE);
    }
#endif

#if 0
    auto path = high_level_grid(maps[texture_index].second, s.source, s.goal, 16);
    for(auto p : path) {
      DrawCircleLines(p.x, p.y, 5.0f, BLUE);
      DrawPixel(p.x, p.y, BLUE);
    }

    for(int i = 0; i < path.size() - 1; i++) {
      auto a = path[i];
      auto b = path[i + 1];
      //DrawLine(a.x, a.y, b.x, b.y, BLUE);
    }


    DrawCircleLines(s.source.x, s.source.y, 10.0f, GREEN);
    DrawPixel(s.source.x, s.source.y, GREEN);
    DrawCircleLines(s.goal.x, s.goal.y, 10.0f, RED);
    DrawPixel(s.goal.x, s.goal.y, GREEN);
#endif

    EndDrawing();
  }

  CloseWindow();

#if 0
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

  test_search<FringeSearch>("Fringe", map, source, goal);
  test_search<FringeSearchSimd>("Fringe Vec", map, source, goal);
#endif
}
