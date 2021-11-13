#include <algorithm>

#include "map.h"
#include "Astar.h"

void a_star_search
  (Map graph,
   Node start,
   Node goal,
   std::vector<Node>& came_from,
   std::vector<double>& cost_so_far)
{
  PriorityQueue<Node, double> frontier;
  frontier.put(start, 0);

  came_from[start] = start;
  cost_so_far[start] = 0;
  
  Point goal_p = graph.node_to_point(goal);
  while (!frontier.empty()) {
    Node current = frontier.get();

    if (current == goal) {
      break;
    }

    Point np = graph.node_to_point(current);
    // For each neighbour
    for(int i = 0; i < 4; i++) {
      Point neigh = neighbour_offsets[i];
      neigh.x += np.x;
      neigh.y += np.y;

      if(neigh.x >= 0 && neigh.x < graph.width && neigh.y >= 0 && neigh.y < graph.height) {
        Node s = graph.point_to_node(neigh);
        if(!graph.data[s]) {
            continue;
        }
        
        double new_cost = cost_so_far[current] + 1;
        if (cost_so_far[s] == -1
            || new_cost < cost_so_far[s]) {
          cost_so_far[s] = new_cost;
          double priority = new_cost + distance(neigh, goal_p);
          frontier.put(s, priority);
          came_from[s] = current;
        }
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

