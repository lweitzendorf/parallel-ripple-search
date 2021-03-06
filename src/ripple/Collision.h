#pragma once

#include "Constants.h"
#include "graph/Map.h"

#include <type_traits>
#include <bitset>


// Struct representing a collision between threads.
struct Collision {
  ThreadId target; // Thread that we collided with
  Node node;       // Node at which the collision happened

  Node parent; // Node that we came from when we found the collision.
               // The other parent of the collision node is stored
               // explicitly in the cache, so we don't need to keep track
               // of it here.

  Collision(){};
  Collision(ThreadId t, Node n, Node p) : target(t), node(n), parent(p) {}
};

class CollisionGraph {
  Map &map;
  Node goal;
  std::vector<ThreadId> neighbors;

  std::bitset<NUM_SEARCH_THREADS> masks[NUM_SEARCH_THREADS] = {};

public:
  std::vector<std::vector<std::pair<ThreadId, Collision>>> graph;

  CollisionGraph(Map &map) : map(map) { }

  void init(Node goal);

  size_t size() const { return graph.size(); }

  std::vector<ThreadId> &neighbours(int i);

  double cost(int i, int j);

  int distance(int i, int j);

  void add_collision(ThreadId source, ThreadId target, Node node, Node parent);

  Collision get_collision(ThreadId t1, ThreadId t2);
};
