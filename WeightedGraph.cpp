#include "WeightedGraph.h"

WeightedGraph::WeightedGraph(int num_vertices) {
  g = weighted_graph_t(num_vertices);
  weights = boost::get(boost::edge_weight, g);
}

WeightedGraph::WeightedGraph(const char* file_name) {
  g = weighted_graph_t(0);
  weights = boost::get(boost::edge_weight, g);

  std::ifstream graph_file(file_name);

  std::string block;
  graph_file >> block;

  if (block != "digraph") {
    std::cout << "Expected digraph, aborting." << std::endl;
    graph_file.close();
    return;
  }
  while (block != "{") {
    graph_file >> block;
  }
  while (true) {
    std::string node_name, label_str, pos_str;
    graph_file >> node_name;

    if (node_name == "subgraph") {
      block = "subgraph";
      break;
    }

    graph_file >> label_str >> pos_str;

    pos_str = pos_str.substr(pos_str.find('"') + 1);
    pos_str = pos_str.substr(0, pos_str.find('"'));
    int x = stoi(pos_str.substr(0, pos_str.find(',')));
    int y = stoi(pos_str.substr(pos_str.find(',')+1));

    boost::add_vertex(g);
    add_location(x, y);
  }

  while (block != "{") {
    graph_file >> block;
  }
  graph_file >> block;

  if (block != "edge") {
    std::cout << "Expected edge, aborting." << std::endl;
    graph_file.close();
    return;
  }

  std::string edge_direction;
  graph_file >> edge_direction;
  edge_direction = edge_direction.substr(edge_direction.find('=') + 1);
  edge_direction = edge_direction.substr(0, edge_direction.find(']'));

  if (edge_direction != "none") {
    std::cout << "Only undirected edges supported." << std::endl;
    graph_file.close();
    return;
  }

  while (true) {
    std::string node_1, direction, node_2;
    graph_file >> node_1;

    if (node_1 == "}")
      break;

    graph_file >> direction >> node_2;

    std::string descriptor = "node";
    vertex_t node_nr_1 = stoi(node_1.substr(descriptor.length()));
    vertex_t node_nr_2 = stoi(node_2.substr(descriptor.length(), node_2.find(';')-descriptor.length()));

    edge_t e = boost::add_edge(node_nr_1, node_nr_2, g).first;
    weights[e] = 1;
  }
  graph_file.close();
}

bool WeightedGraph::add_location(int x, int y) {
  if (locations.size() >= num_vertices())
    return false;

  locations.emplace_back(x, y);
  return true;
}

bool WeightedGraph::add_edge(vertex_t vertex_1, vertex_t vertex_2, int weight) {
  if (vertex_1 >= num_vertices() || vertex_2 >= num_vertices())
    return false;

  edge_t e = boost::add_edge(vertex_1, vertex_2, g).first;
  weights[e] = weight;
  return true;
}

std::list<vertex_t> WeightedGraph::a_star_search(vertex_t start, vertex_t goal) {
  std::vector<vertex_t> p(num_vertices());
  std::vector<int> d(num_vertices());

  try {
    boost::astar_search(g, start, distance_heuristic(locations, goal),
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

