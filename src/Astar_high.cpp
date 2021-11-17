#include <algorithm>
#include "map.h"
#include "Astar.h"
#include <vector>
#include <iostream>

void a_star_search
  (std::vector<Point> &high_nodes,
  std::vector<std::vector<int>> &knn_adj,
   Node start,
   Node goal,
   std::vector<Node>& came_from,
   std::vector<double>& cost_so_far)
{
  PriorityQueue<Node, double> frontier;
  frontier.put(start, 0);

  came_from[start] = start;
  cost_so_far[start] = 0;
  
  Point goal_p = high_nodes[goal];
  while (!frontier.empty()) {
    Node current = frontier.get();

    if (current == goal) {
      break;
    }

    for(auto i : knn_adj[current]){
      Point neigh = high_nodes[i];
      double new_cost = cost_so_far[current] + distance(neigh, high_nodes[current]);
      if(cost_so_far[i] == -1 || new_cost < cost_so_far[i]){
        cost_so_far[i] = new_cost;
        double priority = new_cost + distance(neigh, goal_p);
        frontier.put(i, priority);
        came_from[i] = current;
      }
    }
    
  }
}

std::vector<Node> reconstruct_path(
   Node start, Node goal,
   std::vector<Node>& came_from
) {
  std::vector<Node> path;
  Node current = goal;
  while (current != start) {
    path.push_back(current);
    current = came_from[current];
  }
  path.push_back(start); // optional
  std::reverse(path.begin(), path.end());
  return path;
}

