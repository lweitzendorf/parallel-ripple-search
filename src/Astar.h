#pragma once

#include <queue>
#include <vector>
#include "map.h"

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

void a_star_search
  (Map graph,
   Node start,
   Node goal,
   std::vector<Node>& came_from,
   std::vector<double>& cost_so_far);

std::vector<Node> reconstruct_path(
   Node start, Node goal,
   std::vector<Node>& came_from
);