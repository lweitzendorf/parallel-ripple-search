#pragma once

#include "RippleThread.h"
#include "HighLevelGraph.h"

// Utility class for initializing and invoking ripple search
class RippleSearch {
private:
  Map &map;
  GridHighLevelGraph high_level_graph;
  int working_threads;

  std::vector<concurrent_queue<Message>> message_queues;
  std::vector<RippleCacheNode> cache;
  CollisionGraph collision_graph;

  std::optional<Path<ThreadId>> coordinate_threads();
  std::optional<Path<ThreadId>> check_collision_path();
  bool is_valid_path(Path<Node> &path) const;
  bool is_adjacent_pair(Node n1, Node n2) const;

public:
  // Initialize search on a map
  RippleSearch(Map &map);

  // Start the search from source to goal, they must be valid nodes
  std::optional<Path<Node>> search(Node source, Node goal);

  ThreadId get_owner(Point p) const;
};
