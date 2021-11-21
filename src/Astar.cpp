#include <algorithm>

#include "map.h"
#include "Astar.h"

template<typename Graph, typename Node>
void a_star_search
  (Graph& graph,
   Node start,
   Node goal,
   std::vector<Node>& came_from,
   std::vector<double>& cost_so_far)
{
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
    for(auto it: graph.neighbours(current)) {
      Node s = it.get_node();
      double new_cost = cost_so_far[current] + it.get_cost();

      if (cost_so_far[s] == -1 || new_cost < cost_so_far[s]) {
        cost_so_far[s] = new_cost;
        double priority = new_cost + graph.distance(s, goal);
        frontier.put(s, priority);
        came_from[s] = current;
      }
    }
  }
}

#if 0
Point neigh = neighbour_offsets[i];
      neigh.x += np.x;
      neigh.y += np.y;

      if(neigh.x >= 0 && neigh.x < graph.width && neigh.y >= 0 && neigh.y < graph.height) {
        Node s = graph.point_to_node(neigh);
        if(!graph.data[s]) {
            continue;
        }
#endif

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

