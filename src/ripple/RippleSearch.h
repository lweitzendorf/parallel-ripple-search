#pragma once

#include "RippleThread.h"

// Utility class for initializing and invoking ripple search
class RippleSearch {
private:
  Map &map;
  std::vector<concurrent_queue<Message>> message_queues;
  std::vector<std::atomic<ThreadId>> node_owners;
  CollisionGraph collision_graph;
  Node source;
  Node goal;

  std::optional<Path<ThreadId>> coordinate_threads();
  std::optional<Path<ThreadId>> check_collision_path();
  bool is_valid_path(Path<Node> &path) const;
  bool is_adjacent_pair(Node n1, Node n2) const;

  public:
  // Initialize search on a map
  RippleSearch(Map &map, Node source, Node goal);

  // Start the search from source to goal, they must be valid nodes
  std::optional<Path<Node>> search();

  ThreadId get_owner(Point p) const;
};
