#pragma once

#include "Map.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <list>
#include <string>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              boost::no_property,
                              boost::property<boost::edge_weight_t, int>>
    weighted_graph_t;
typedef boost::property_map<weighted_graph_t, boost::edge_weight_t>::type
    weight_map_t;
typedef boost::graph_traits<weighted_graph_t>::edge_descriptor edge_t;
typedef boost::graph_traits<weighted_graph_t>::vertex_descriptor Node;
typedef boost::graph_traits<weighted_graph_t>::vertices_size_type vertices_size_t;
typedef boost::graph_traits<weighted_graph_t>::edges_size_type edges_size_t;

typedef boost::graph_traits<weighted_graph_t>::out_edge_iterator out_edge_iterator;

template <typename T> using Path = std::vector<T>;

class WeightedGraph {
public:
  WeightedGraph();

  Node add_vertex(Point);
  std::optional<edge_t> add_edge(Node, Node, int);

  vertices_size_t num_vertices() const { return boost::num_vertices(g); }
  edges_size_t num_edges() const { return boost::num_edges(g); }

  size_t size() { return num_vertices(); }
  size_t width() const { return num_vertices() ? (max_x - min_x + 1) : 0; }
  size_t height() const { return num_vertices() ? (max_y - min_y + 1) : 0; }

  std::optional<Node> point_to_vertex(Point);
  std::optional<Point> vertex_to_point(Node);

  bool is_reachable(Point);

  std::vector<Node> neighbors(Node);

  double cost(Node, Node);
  double distance(Node, Node);

  std::list<Node> a_star_search(Node, Node);

  void load_from_image(std::string file_name);

  static std::function<bool(Node)> node_eq_predicate(Node n1) {
    return [=](Node n2) -> bool { return n1 == n2; };
  };

private:
  weighted_graph_t g;
  weight_map_t weights;
  std::vector<Point> locations;
  std::unordered_map<Point, Node> location_to_vertex;

  int min_x = 0, max_x = 0;
  int min_y = 0, max_y = 0;

  struct found_goal {};

  class astar_goal_visitor : public boost::default_astar_visitor {
  public:
    explicit astar_goal_visitor(Node goal) : m_goal(goal) {}
    template <class Graph> void examine_vertex(Node u, Graph &g) {
      if (u == m_goal)
        throw found_goal();
    }

  private:
    Node m_goal{};
  };

  class euclidean_distance_heuristic
      : public boost::astar_heuristic<weighted_graph_t, double> {
  public:
    euclidean_distance_heuristic(std::vector<Point> l, Node goal)
        : m_location(std::move(l)), m_goal(goal) {}
    double operator()(Node u) {
      int dx = m_location[m_goal].x - m_location[u].x;
      int dy = m_location[m_goal].y - m_location[u].y;
      return std::sqrt(dx * dx + dy * dy);
    }

  private:
    std::vector<Point> m_location;
    Node m_goal;
  };

  class manhattan_distance_heuristic
      : public boost::astar_heuristic<weighted_graph_t, int> {
  public:
    manhattan_distance_heuristic(std::vector<Point> l, Node goal)
        : m_location(std::move(l)), m_goal(goal) {}
    int operator()(Node u) {
      int dx = m_location[m_goal].x - m_location[u].x;
      int dy = m_location[m_goal].y - m_location[u].y;
      return std::abs(dx) + std::abs(dy);
    }

  private:
    std::vector<Point> m_location;
    Node m_goal;
  };
};

