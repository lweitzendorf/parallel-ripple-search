#include "WeightedGraph.h"
#include<iterator>

WeightedGraph::WeightedGraph() {
  g = weighted_graph_t(0);
  weights = boost::get(boost::edge_weight, g);
}

vertex_t WeightedGraph::add_vertex(int x, int y) {
  locations.emplace_back(x, y);
  return boost::add_vertex(g);
}

bool WeightedGraph::add_edge(vertex_t vertex_1, vertex_t vertex_2, int weight) {
  if (vertex_1 >= num_vertices() || vertex_2 >= num_vertices())
    return false;

  edge_t e = boost::add_edge(vertex_1, vertex_2, g).first;
  weights[e] = weight;
  return true;
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

std::list<vertex_t> WeightedGraph::a_star_search(vertex_t start, vertex_t goal) {
  if (start < 0 || goal < 0 || start >= num_vertices() || goal >= num_vertices())
    return { };

  std::vector<vertex_t> p(num_vertices());
  std::vector<int> d(num_vertices());

  try {
    boost::astar_search(g, start, manhattan_distance_heuristic(locations, goal),
                        boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(astar_goal_visitor(goal)));
  } catch (found_goal fg) {
    std::list<vertex_t> shortest_path = { goal };
    for (vertex_t v = p[goal]; v != shortest_path.front(); v = p[v]) {
      shortest_path.push_front(v);
    }
    return shortest_path;
  }
  return { };
}

Map WeightedGraph::create_map() {
  int width = 0, height = 0;

  for (const auto loc : locations) {
    width = std::max(width, loc.x + 1);
    height = std::max(height, loc.y + 1);
  }

  Map map(width, height);

  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++) {
      map.set(Point(x, y), 0);
    }
  }

  for (int v = 0; v < num_vertices(); v++) {
    map.set(locations[v], boost::in_degree(v, g) != 0);
  }

  return map;
}

