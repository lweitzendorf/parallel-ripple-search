#include <queue>
#include <unordered_map>
#include "map.h"

namespace std {
/* implement hash function so we can put GridLocation into an unordered_set */
template <> struct hash<Point> {
  std::size_t operator()(const Point& id) const noexcept {
    // NOTE: better to use something like boost hash_combine
    return std::hash<int>()(id.x ^ (id.y << 16));
  }
};
}

template<typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                 std::greater<PQElement>> elements;

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

inline double heuristic(Point a, Point b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

template<typename Location, typename Graph>
void a_star_search
  (Graph graph,
   Location start,
   Location goal,
   std::unordered_map<Location, Location>& came_from,
   std::unordered_map<Location, double>& cost_so_far)
{
  PriorityQueue<Location, double> frontier;
  frontier.put(start, 0);

  came_from[start] = start;
  cost_so_far[start] = 0;
  
  while (!frontier.empty()) {
    Location current = frontier.get();

    if (current == goal) {
      break;
    }

    // For each neighbour
    for(int i = 0; i < 4; i++) {
      Location neigh = neighbour_offsets[i];
      neigh.x += current.x;
      neigh.y += current.y;

      if(neigh.x >= 0 && neigh.x < graph.width && neigh.y >= 0 && neigh.y < graph.height) {
        Node s = graph.point_to_node(neigh);
        if(!graph.data[s]) {
            continue;
        }
        
        Location next = neigh;
        double new_cost = cost_so_far[current] + 1;
        if (cost_so_far.find(next) == cost_so_far.end()
            || new_cost < cost_so_far[next]) {
          cost_so_far[next] = new_cost;
          double priority = new_cost + heuristic(next, goal);
          frontier.put(next, priority);
          came_from[next] = current;
        }
      }
    }
  }
}

template<typename Location>
std::vector<Location> reconstruct_path(
   Location start, Location goal,
   std::unordered_map<Location, Location> came_from
) {
  std::vector<Location> path;
  Location current = goal;
  while (current != start) {
    path.push_back(current);
    current = came_from[current];
  }
  path.push_back(start); // optional
  std::reverse(path.begin(), path.end());
  return path;
}

