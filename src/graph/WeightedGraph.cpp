#include "WeightedGraph.h"
#include <iterator>

WeightedGraph::WeightedGraph() {
  g = weighted_graph_t(0);
  weights = boost::get(boost::edge_weight, g);
}

vertex_t WeightedGraph::add_vertex(Point p) {
  vertex_t v = boost::add_vertex(g);
  locations.push_back(p);
  location_to_vertex[p] = v;

  max_x = std::max(max_x, p.x);
  min_x = std::min(min_x, p.x);
  max_y = std::max(max_y, p.y);
  min_y = std::min(min_y, p.y);

  return v;
}

std::optional<edge_t> WeightedGraph::add_edge(vertex_t vertex_1, vertex_t vertex_2, float weight) {
  if (vertex_1 >= num_vertices() || vertex_2 >= num_vertices())
    return std::nullopt;

  edge_t e = boost::add_edge(vertex_1, vertex_2, g).first;
  weights[e] = weight;
  return e;
}

std::optional<vertex_t> WeightedGraph::point_to_vertex(Point p) {
  if (location_to_vertex.contains(p)) {
    return location_to_vertex[p];
  }
  return std::nullopt;
}

auto WeightedGraph::neighbors(vertex_t vertex) {
  auto out_edges = boost::out_edges(vertex, g);
  std::vector<vertex_t> out(out_edges.second - out_edges.first);
  return std::transform(out_edges.first, out_edges.second, out.begin(),
                        [&](auto edge) -> vertex_t {
                          auto s = boost::source(edge, g);
                          auto t = boost::target(edge, g);
                          return s == vertex ? t : s;
                        });
}

std::optional<Path<Node>> WeightedGraph::a_star_search(vertex_t start, vertex_t goal) {
  if (start < 0 || goal < 0 || start >= num_vertices() ||
      goal >= num_vertices())
    return {};

  std::vector<vertex_t> p(num_vertices());
  std::vector<int> d(num_vertices());

  try {
    boost::astar_search(
        g, start, euclidean_distance_heuristic(locations, goal),
        boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(
            astar_goal_visitor(goal)));
  } catch (found_goal fg) {
    Path<Node> path;
    Node current = goal;
    while (current != start) {
      path.push_back(current);
      current = p[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
  }
  return {};
}

Map WeightedGraph::create_map() {
  int width = 0, height = 0;

  for (const auto loc : locations) {
    width = std::max(width, loc.x + 1);
    height = std::max(height, loc.y + 1);
  }

  Map map(width, height);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      map.set(Point(x, y), 0);
    }
  }

  for (int v = 0; v < num_vertices(); v++) {
    map.set(locations[v], boost::in_degree(v, g) != 0);
  }

  return map;
}


void WeightedGraph::build_from_map(Map &map) {
  for (int y = 0; y < map.height(); y++) {
    for (int x = 0; x < map.width(); x++) {
      Point p(x, y);
      add_vertex(p);
      Node n = map.point_to_node(p);

      if (map.get(Point(x, y))) {
        for (auto offset : Map::neighbour_offsets) {
          Point neighbor = p + offset;

          if (map.in_bounds(neighbor) && map.get(neighbor)) {
            Node neighbor_node = map.point_to_node(neighbor);
            if (neighbor_node < n) {
              float cost = neighbor.x == p.x || neighbor.y == p.y ? 1 : sqrtf(2);
              add_edge(neighbor_node, n, cost);
            }
          }
        }
      }
    }
  }
}