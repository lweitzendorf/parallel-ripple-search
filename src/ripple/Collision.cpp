#include "Collision.h"

void CollisionGraph::init(Node g) {
  graph.clear();
  graph.resize(NUM_SEARCH_THREADS);

  neighbors.clear();

  for(int i = 0; i < NUM_SEARCH_THREADS; i++) {
    masks[i] = 0;
  }
  
  goal = g;
}

void CollisionGraph::add_collision(ThreadId source, ThreadId target, Node node, Node parent) {
  assert(source != target);

  // Make sure we only store one collision for each pair of threads
  if (!(masks[source] & (1 << target) || masks[target] & (1 << source))) {
    graph[source].push_back(
            std::make_pair(target, Collision(target, node, parent)));
    graph[target].push_back(
            std::make_pair(source, Collision(target, node, parent)));

    masks[source] |= (1 << target);
    masks[target] |= (1 << source);
  }
}

Collision CollisionGraph::get_collision(ThreadId t1, ThreadId t2) {
  for (auto &c : graph[t1]) {
    if (c.first == t2) {
      return c.second;
    }
  }

  printf("get_collision failed with %d -> %d\n", t1, t2);
  assert(false);
  return {};
}

std::vector<ThreadId> &CollisionGraph::neighbours(int i) {
  neighbors.clear();
  std::transform(graph[i].begin(), graph[i].end(), std::back_inserter(neighbors),
                 [](std::pair<ThreadId, Collision> &c) { return c.first; });
  return neighbors;
}

// TODO I know the below are wrong
double CollisionGraph::cost(int i, int j) {
  return 1;
}

int CollisionGraph::distance(int i, int j) {
  return 0;
}
