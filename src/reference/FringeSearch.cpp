#include "FringeSearch.h"

#include <climits>

std::optional<std::vector<Node>> FringeSearch::search() {
  cache.clear();
  cache.resize(graph.num_vertices(), {});

  fringe_list.clear();

  // Add source node to fringe list
  fringe_list.push_front(source);
  cache[source].in_list = true;
  cache[source].cost = 0;
  cache[source].parent = source;
  cache[source].list_entry = fringe_list.begin();
  cache[source].visited = true;

  // Lambda for computing heuristic
  auto h = [&](Node i) {
    return graph.distance(i, goal);
  };

  int flimit = h(source);
  int found = false;
  auto nnode = fringe_list.end();

  // int it = 0;
  while (!found && !fringe_list.empty()) {
    //std::cout << "Iteration: " << it++ << std::endl;
    int fmin = INT_MAX;
    nnode = fringe_list.begin();

    do {

      if (*nnode == goal) {
        found = true;
        break;
      }

      auto &n = cache[*nnode];

      int g = n.cost;
      int f = g + h(*nnode);

      if (f > flimit) {
        fmin = std::min(fmin, f);
        nnode++;
        continue;
      }

      // For each neighbour
      for (auto &s: graph.neighbors(*nnode)) {
        int gs = g + 1;
        if (cache[s].visited) {
          int gi = cache[s].cost;
          if (gs >= gi) {
            continue;
          }
        }

        if (cache[s].in_list) {
          fringe_list.erase(cache[s].list_entry);
          cache[s].in_list = false;
        }

        cache[s].visited = true;
        cache[s].cost = gs;
        cache[s].parent = *nnode;
        //cache[s].h = h(s);

        fringe_list.insert(std::next(nnode), s);

        cache[s].list_entry = std::next(nnode);
        cache[s].in_list = true;
      }

      cache[*nnode].in_list = false;

      auto tmp = std::next(nnode);
      fringe_list.erase(nnode);
      nnode = tmp;
    } while (nnode != fringe_list.end());

    flimit = fmin;
  }

  if (nnode != fringe_list.end()) {
    std::vector<Node> shortest_path = {goal};
    for (Node v = cache[goal].parent; v != source; v = cache[v].parent) {
      shortest_path.push_back(v);
    }
    shortest_path.push_back(source);
    std::reverse(shortest_path.begin(), shortest_path.end());
    return std::optional<Path<Node>>{shortest_path};
  } else {
    return std::nullopt;
  }
}

FringeSearch::FringeSearch(WeightedGraph &graph, Node source, Node goal) :
        graph(graph), source(source), goal(goal) {
}
