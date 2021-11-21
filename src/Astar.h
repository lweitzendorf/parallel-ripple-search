#pragma once

#include <queue>
#include <vector>
#include "map.h"

template<typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

  inline bool empty() const {
     return elements.empty();
  }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};


template<typename Graph, typename Node>
Path a_star_search_gen(Graph& graph, Node start, Node goal)
{
  Path came_from(graph.size());
  std::vector<double> cost_so_far(graph.size(), -1);

  PriorityQueue<Node, double> frontier;
  frontier.put(start, 0);

  came_from[start] = start;
  cost_so_far[start] = 0;
  
  while (!frontier.empty()) {
    Node current = frontier.get();

    if (current == goal) {
      break;
    }

    // For each neighbour
    for(Node s: graph.neighbours(current)) {
      double new_cost = cost_so_far[current] + graph.cost(current, s);

      if (cost_so_far[s] == -1 || new_cost < cost_so_far[s]) {
        cost_so_far[s] = new_cost;
        double priority = new_cost + graph.distance(s, goal);
        frontier.put(s, priority);
        came_from[s] = current;
      }
    }
  }

  return came_from;
}

template<typename Node>
Path reconstruct_path_gen(Node start, Node goal, Path& came_from) {
  Path path;
  Node current = goal;
  while (current != start) {
    path.push_back(current);
    current = came_from[current];
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}