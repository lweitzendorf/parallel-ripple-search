#pragma once

#include "graph/Map.h"
#include "utility/PriorityQueue.h"
#include <optional>
#include <vector>

template <typename Node>
  Path<Node> reconstruct_path(Node start, Node goal, Path<Node> &came_from) {
  Path<Node> path;
  Node current = goal;
  while (current != start) {
    path.push_back(current);
    current = came_from[current];
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

template <typename Graph, typename Node>
std::optional<Path<Node>> a_star_search(Graph &graph, Node start, Node goal) {
  Path<Node> came_from(graph.size());
  std::vector<std::optional<float>> cost_so_far(graph.size(), std::nullopt);

  PriorityQueue<Node, float> frontier;
  frontier.put(start, 0);

  came_from[start] = start;
  cost_so_far[start] = 0;

  while (!frontier.empty()) {
    Node current = frontier.get();

    if (current == goal)
      return reconstruct_path(start, goal, came_from);

    // For each neighbour
    for (Node s : graph.neighbours(current)) {
      float new_cost = cost_so_far[current].value() + graph.cost(current, s);

      if (!cost_so_far[s] || new_cost < cost_so_far[s].value()) {
        cost_so_far[s] = new_cost;
        float priority = new_cost + graph.distance(s, goal);
        frontier.put(s, priority);
        came_from[s] = current;
      }
    }
  }
  return {};
}

class AStarSearch {
  private:
  Map &map;

  public:
  AStarSearch(Map &map) : map(map) {}

  std::optional<Path<Node>> search( Node source, Node goal) {
    return a_star_search(map, source, goal);
  }
};
