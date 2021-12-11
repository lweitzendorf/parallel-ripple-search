#pragma once

#include "graph/map.h"
#include "reference/fringe.h"
#include "utility/Timer.h"
#include "Collision.h"
#include "Thread.h"
#include "Message.h"
#include "RippleThread.h"

/** NOTE s for refactoring:
 * there should exist two types of threads
 *
 * 1. a coordinator
 * 2. workers
 *
 * (1) is not unlike the master thread, however, this thread is going to
 *     make sure that the other threads are apporpriately messaged and
 *     working on the correct thing.
 *
 * (2) is the basic worker thread from before. These should be searching for
 *     paths and /that's it/. (1) can reconstruct paths while the others work
 *     on phase 2 (if we want) but mostly it should be coordinating messages
 *     on collisions. (2) should not have fields for things like the
 *     CollisionGraph that is only needed by one thread.
 *
 **/

// Utility class for initializing and invoking ripple search
class RippleSearch {
  private:
    Map &map;
    std::vector<concurrent_queue<Message>> message_queues;
    std::vector<RippleCacheNode> cache;
    CollisionGraph collision_graph;
    Node source;
    Node goal;

  public:
    // Initialize search on a map
    RippleSearch(Map &map, Node source, Node goal);

    // Start the search from source to goal, they must be valid nodes
    std::optional<Path<Node>> search();

    ThreadId get_owner(Point p) const;
};
