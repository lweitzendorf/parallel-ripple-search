#include "WeightedGraph.h"

#include <raylib.h>

WeightedGraph::WeightedGraph() {
  g = weighted_graph_t(0);
  weights = boost::get(boost::edge_weight, g);
}

Node WeightedGraph::add_vertex(Point p) {
  Node v = boost::add_vertex(g);
  locations.push_back(p);
  location_to_vertex[p] = v;

  max_x = std::max(max_x, p.x);
  min_x = std::min(min_x, p.x);
  max_y = std::max(max_y, p.y);
  min_y = std::min(min_y, p.y);

  return v;
}

std::optional<edge_t> WeightedGraph::add_edge(Node vertex_1, Node vertex_2, int weight) {
  if (vertex_1 >= num_vertices() || vertex_2 >= num_vertices())
    return std::nullopt;

  edge_t e = boost::add_edge(vertex_1, vertex_2, g).first;
  weights[e] = weight;
  return e;
}

std::optional<Node> WeightedGraph::point_to_vertex(Point p) {
  if (location_to_vertex.contains(p)) {
    return location_to_vertex[p];
  }
  return std::nullopt;
}

std::optional<Point> WeightedGraph::vertex_to_point(Node v) {
  if (v < num_vertices()) {
    return locations[v];
  }
  return std::nullopt;
}

std::vector<Node> WeightedGraph::neighbors(Node vertex) {
  auto out_edges = boost::out_edges(vertex, g);
  std::vector<Node> out(out_edges.second - out_edges.first);
  std::transform(out_edges.first, out_edges.second, out.begin(),
                        [&](auto edge) -> Node {
                          Node s = boost::source(edge, g);
                          Node t = boost::target(edge, g);
                          return s == vertex ? t : s;
                        });
  return out;
}

std::list<Node> WeightedGraph::a_star_search(Node start,
                                             Node goal) {
  if (start < 0 || goal < 0 || start >= num_vertices() ||
      goal >= num_vertices())
    return {};

  std::vector<Node> p(num_vertices());
  std::vector<int> d(num_vertices());

  try {
    boost::astar_search(
        g, start, euclidean_distance_heuristic(locations, goal),
        boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(
            astar_goal_visitor(goal)));
  } catch (found_goal fg) {
    std::list<Node> shortest_path = {goal};
    for (Node v = p[goal]; v != shortest_path.front(); v = p[v]) {
      shortest_path.push_front(v);
    }
    return shortest_path;
  }
  return {};
}

bool WeightedGraph::is_reachable(Point p) {
  if (location_to_vertex.contains(p)) {
    Node v = location_to_vertex[p];
    return (boost::in_degree(v, g) > 0);
  }
  return false;
}

double WeightedGraph::cost(Node n1, Node n2) {
  return distance(n1, n2);
}

double WeightedGraph::distance(Node n1, Node n2) {
  int dx = locations[n2].x - locations[n1].x;
  int dy = locations[n2].y - locations[n1].y;
  return std::sqrt(dx * dx + dy * dy);
}

void WeightedGraph::load_from_image(std::string file_path) {
  Image img = LoadImage(file_path.c_str());

  for (int y = 0; y < img.height; y++) {
    for (int x = 0; x < img.width; x++) {
      Point p(x, y);
      Node n = add_vertex(p);

      if (GetImageColor(img, x, y).r != 0) {
        for (auto offset : Map::neighbour_offsets) {
          Point neighbor = p + offset;

          if (is_reachable(p) && is_reachable(neighbor)) {
            Node neighbor_node = point_to_vertex(neighbor).value();
            add_edge(neighbor_node, n, 1);
          }
        }
      }
    }
  }

  UnloadImage(img);
}

