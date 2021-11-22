#ifndef DPHPC_WEIGHTEDGRAPH_H
#define DPHPC_WEIGHTEDGRAPH_H

#include "../src/map.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <iostream>
#include <list>
#include <cmath>
#include <string>
#include <fstream>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              boost::no_property, boost::property<boost::edge_weight_t, int>> weighted_graph_t;
typedef boost::property_map<weighted_graph_t, boost::edge_weight_t>::type weight_map_t;
typedef boost::graph_traits<weighted_graph_t>::edge_descriptor edge_t;
typedef boost::graph_traits<weighted_graph_t>::vertex_descriptor vertex_t;
typedef boost::graph_traits<weighted_graph_t>::vertices_size_type vertices_size_t;
typedef boost::graph_traits<weighted_graph_t>::edges_size_type edges_size_t;

class WeightedGraph {
public:
    WeightedGraph();

    vertex_t add_vertex(int, int);
    bool add_edge(vertex_t, vertex_t, int);

    vertices_size_t num_vertices() { return boost::num_vertices(g); }
    edges_size_t num_edges() { return boost::num_edges(g); }

    std::list<vertex_t> a_star_search(vertex_t, vertex_t);

    Map create_map();

private:
    weighted_graph_t g;
    weight_map_t weights;
    std::vector<Point> locations;

    struct found_goal {};

    class astar_goal_visitor : public boost::default_astar_visitor
    {
      public:
          explicit astar_goal_visitor(vertex_t goal) : m_goal(goal) {}
          template <class Graph>
          void examine_vertex(vertex_t u, Graph& g) {
            if(u == m_goal)
              throw found_goal();
          }
      private:
          vertex_t m_goal{};
    };

    class euclidean_distance_heuristic : public boost::astar_heuristic<weighted_graph_t, int>
    {
      public:
        euclidean_distance_heuristic(std::vector<Point> l, vertex_t goal) : m_location(std::move(l)), m_goal(goal) {}
            int operator()(vertex_t u) {
              int dx = m_location[m_goal].x - m_location[u].x;
              int dy = m_location[m_goal].y - m_location[u].y;
              return dx*dx + dy*dy;
            }
      private:
          std::vector<Point> m_location;
          vertex_t m_goal;
    };

  class manhattan_distance_heuristic : public boost::astar_heuristic<weighted_graph_t, int>
  {
    public:
    manhattan_distance_heuristic(std::vector<Point> l, vertex_t goal) : m_location(std::move(l)), m_goal(goal) {}
    int operator()(vertex_t u) {
      int dx = m_location[m_goal].x - m_location[u].x;
      int dy = m_location[m_goal].y - m_location[u].y;
      return std::abs(dx) + std::abs(dy);
    }
    private:
    std::vector<Point> m_location;
    vertex_t m_goal;
  };
};

#endif //DPHPC_WEIGHTEDGRAPH_H
