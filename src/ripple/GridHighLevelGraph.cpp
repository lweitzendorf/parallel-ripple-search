#include "utility/Timer.h"
#include "reference/Astar.h"
#include "ripple/HighLevelGraph.h"

#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>

using glm::vec2;

// Width and height of the grid that is sampled to create the high level graph
const int GRID_SIZE = 64;

GridHighLevelGraph::GridHighLevelGraph(Map& map): map(map), grid(GRID_SIZE, GRID_SIZE), adj(GRID_SIZE * GRID_SIZE) {
  cell = vec2((float)map.width() / (GRID_SIZE), (float)map.height() / (GRID_SIZE));
  
  // Initialize grid
  for(int y = 0; y < GRID_SIZE; y++) {
    for(int x = 0; x < GRID_SIZE; x++) {
      vec2 v = grid_to_map(vec2(x, y));
      Point p(v.x, v.y);
      if(map.get(p)) {
        grid.set(Point(x, y), 1);
      } else {
        grid.set(Point(x, y), 0);
      }
    }
  }


  // Connect neighbours
  for(int y = 0; y < GRID_SIZE; y++) {
    for(int x = 0; x < GRID_SIZE; x++) {
      Point p(x, y);
      if(!grid.get(p)) {
        continue;
      }

      Node n = grid.point_to_node(p);

      // x neighbour
      for(int x_before = x - 1; x_before >= 0; x_before--) {
        Point p_before(x_before, y);

        if(!grid.get(p_before)) {
          continue;
        }

        Node n_before = grid.point_to_node(p_before);
        adj[n].push_back(n_before);
        adj[n_before].push_back(n);
        break;
      }

      // y neighbour
      for(int y_before = y - 1; y_before >= 0; y_before--) {
        Point p_before(x, y_before);

        if(!grid.get(p_before)) {
          continue;
        }

        Node n_before = grid.point_to_node(p_before);
        adj[n].push_back(n_before);
        adj[n_before].push_back(n);
        break;
      }

      // TODO: also search in these directions
      // diagonal neighbours
      Point diag1 = Point(x - 1, y - 1);
      Point diag2 = Point(x + 1, y - 1);
      if(grid.in_bounds(diag1) && grid.get(diag1)) {
        Node nd = grid.point_to_node(diag1);
        adj[nd].push_back(n);
        adj[n].push_back(nd);
      }

      if(grid.in_bounds(diag2) && grid.get(diag2)) {
        Node nd = grid.point_to_node(diag2);
        adj[nd].push_back(n);
        adj[n].push_back(nd);
      }
    }
  }
}

double GridHighLevelGraph::cost(int i, int j) { 
  Point pi = grid.node_to_point(i);
  Point pj = grid.node_to_point(j);

  float dist = glm::distance2(vec2(pi.x, pi.y), vec2(pj.x, pj.y));

  // Weigh heavily non-neighbouring nodes on the grid
  double cost = 0;
  if(dist >= 2.1) {
    cost = dist * 1000;
  }

  return cost + grid.distance(i, j);
}

int GridHighLevelGraph::distance(int i, int j) { 
  return grid.distance(i, j);
}


std::vector<Point> GridHighLevelGraph::refine_high_level_path(const std::vector<Point>& path, int num) {
  if(num <= 0) {
    return {};
  }

  if(path.size() <= num) {
    return path;
  }

  float total_length = 0;
  for(size_t i = 0; i < path.size() - 1; i++) {
    Point u = path[i];
    Point v = path[i + 1];
    total_length += glm::distance(vec2(u.x, u.y), vec2(v.x, v.y));
  }

  float segment_length = total_length / (num + 1);
  float target_length = segment_length;
  float length_so_far = 0;

  std::vector<Point> result;
  for(size_t i = 0; i < path.size() - 1; i++) {
    Point u = path[i];
    Point v = path[i + 1];
    float dist = glm::distance(vec2(u.x, u.y), vec2(v.x, v.y));
    if(length_so_far + dist >= target_length && (result.empty() || result.back() != v)) {
      result.push_back(v);
      target_length += segment_length;
    }

    length_so_far += dist;

    if(result.size() == num) {
      break;
    }
  }

  return result;
}

Point GridHighLevelGraph::get_closest_point_on_grid(Point point) {
  Point closest;
  float dist = FLT_MAX;

  vec2 pointv = vec2(point.x, point.y);
  for(int y = 0; y < GRID_SIZE; y++) {
    for(int x = 0; x < GRID_SIZE; x++) {
      vec2 v = grid_to_map(vec2(x, y));
      Point p(v.x, v.y);
      if(map.get(p)) {
        grid.set(Point(x, y), 1);

        float ds = glm::distance2(v, pointv);
        if(ds < dist) {
          dist = ds;
          closest = Point(x, y);
        }
      }
    }
  }

  return closest;
}

std::vector<Point> GridHighLevelGraph::get_full_path(Point source, Point goal) {
  Point closest_source = get_closest_point_on_grid(source);
  Point closest_goal = get_closest_point_on_grid(goal);

  auto grid_path_opt = a_star_search(*this, grid.point_to_node(closest_source), grid.point_to_node(closest_goal));

  if(grid_path_opt.has_value()) {
    std::vector<Point> points;
    points.reserve(grid_path_opt.value().size());

    for(auto n: grid_path_opt.value()) {
      Point point = grid.node_to_point(n);
      points.emplace_back(grid_to_map(point));
    }

    return points;
  } else {
    assert(false);
    return {};
  }
}

Path<Node> GridHighLevelGraph::create_high_level_path(Node source, Node goal, int num) {
  auto full_path = get_full_path(map.node_to_point(source), map.node_to_point(goal));
  auto refined_path = refine_high_level_path(full_path, num - 2);

  std::vector<Node> result;
  result.reserve(refined_path.size() + 2);

  result.push_back(source);
  for(auto p: refined_path) {
    result.push_back(map.point_to_node(p));
  }
  if(result.empty() || result.back() != goal)
    result.push_back(goal);

  return result;
}

vec2 GridHighLevelGraph::grid_to_map(vec2 p) {
  return cell * p + cell * 0.5f;
}

Point GridHighLevelGraph::grid_to_map(Point p) {
  vec2 v = grid_to_map(vec2(p.x, p.y));
  return Point(v.x, v.y);
}
