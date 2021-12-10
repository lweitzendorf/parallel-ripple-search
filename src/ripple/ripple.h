#pragma once

#include <atomic>
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <stdint.h>
#include <vector>

#include <oneapi/tbb/concurrent_queue.h>

#include "graph/map.h"
#include "reference/fringe.h"
#include "utility/Timer.h"

#define NUM_THREADS 4

using oneapi::tbb::concurrent_queue;

enum ThreadId : int8_t {
  THREAD_NONE = -1,
  THREAD_SOURCE = 0,
  THREAD_GOAL = NUM_THREADS - 1,
};

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

  uint32_t masks[NUM_THREADS] = {};
  static_assert(sizeof(masks[0]) * 8 >= NUM_THREADS);

public:
  std::vector<std::vector<std::pair<ThreadId, Collision>>> graph;

  CollisionGraph(Map &map, Node goal) : map(map), goal(goal) {
    graph.resize(NUM_THREADS);
  }

  size_t size() { return graph.size(); }

  std::vector<ThreadId> &neighbours(int i) {
    neighbors.clear();
    std::transform(graph[i].begin(), graph[i].end(), back_inserter(neighbors),
                   [](std::pair<ThreadId, Collision> &c) { return c.first; });
    return neighbors;
  }

  // TODO I know the below are wrong
  double cost(int i, int j) {
    // return map.distance(graph[i][i].collision_node, goal);
    return 1;
  }

  int distance(int i, int j) {
    // return map.distance(graph[i][i].collision_node, goal);
    return 0;
  }

  void add_collision(ThreadId source, ThreadId target, Node node, Node parent) {
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

  Collision get_collision(ThreadId t1, ThreadId t2) {
    for (int i = 0; i < graph[t1].size(); i++) {
      if (graph[t1][i].first == t2) {
        return graph[t1][i].second;
      }
    }

    printf("get_collision failed with %d -> %d\n", t1, t2);
    assert(0);
    return {};
  }
};

// Message type
enum MessageType {
  MESSAGE_COLLISION, // Sent by other threads to the source thread during phase
                     // 1 when a collision happens
  MESSAGE_PHASE_2,   // Sent by the source thread to signal other threads that
                     // phase 2 has started
  MESSAGE_STOP, // Sent by the source thread to signal an other thread that it
                // has no work to do in phase 2
  MESSAGE_DONE, // Sent by other threads to the source thread to signal that
                // they finished their phase 2 work
};

// Struct representing a message that is sent between ripple threads
// the valid fields of the union depend on the message type
struct Message {
  MessageType type;
  union {
    struct {
      Node source;
      Node target;
    }; // type = MESSAGE_PHASE_2 for slave threads

    Node final_node; // type = MESSAGE_PHASE_2 for goal thread

    struct {
      Node collision_node;   // Node that is already owned by the target thread
      Node collision_parent; // Node that the source thread came from when he
                             // found the collision
      ThreadId collision_source; // Thread that found the collision
      ThreadId collision_target; // Thread that already owned the node
    };                           // type = MESSAGE_COLLISION
  };
};

// Question: can we put some of these enums within the class? is this a better
// design?
enum FringeInterruptAction { NONE, RESET, EXIT };

// TODO: test aligning this struct to cache line size to avoid
// false sharing between threads that are exploring neighbouring nodes
struct RippleCacheNode {
  // Id of the thread that first discovered the Node and thus owns it
  std::atomic<ThreadId> thread;

  // Fringe search data
  FringeNode node;
};

// Class representing a thread of the ripple search algorithm
class RippleThread {
public:
  double time_first;
  double time_second;
  Timer timer;

private:
  std::unique_ptr<std::thread> thread;

  // Read only data:
  // My id
  ThreadId id;
  // internal thread

  // Map
  Map &map;
  // Message queues for sending to other threads
  std::vector<concurrent_queue<Message>> &message_queues;

  // Condition variable and mutex for putting the thread to sleep
  // when it has to wait for other threads
  std::mutex wait_mutex;
  std::condition_variable wait_cv;

  // Current source ang goals for the thread.
  // Slave threads have 2 goals because they search in both directions
  // until they have a collision with one of the two.
  // goal_2 == INVALID_NODE for the Source and Goal threads
  Node source;
  Node goal;
  Node goal_2;

  // Fringe search info
  FringeList fringe_list;
  std::function<int(RippleThread *, Node)> heuristic;
  int flimit;

  // Reference to vector of node info for fringe search, shared between all
  // threads
  std::vector<RippleCacheNode> &cache;

  // Adjacency list of the graph of all the nodes in which we have a collision
  // collision_graph[id1] -> list of all collisions of thread id1 with other
  // threads The adjacency list is kept symmetric such that if there is a
  // collision from id1 to id2 it also appears as a collision from id2 to id1.
  // This graph is only read and written by the source thread
  CollisionGraph &collision_graph;

  // Mask of threads that we collided with, the i-th bit is 1 if we collided
  // with the i-th thread
  uint32_t collision_mask = 0;
  static_assert(sizeof(collision_mask) * 8 >= NUM_THREADS);

  // Phase 2 flag, set to true when we enter phase 2
  bool phase2 = false;

  // Segment of the final path owned by the thread
  Path<Node> final_path;

  // Path of threads contributing to phase 2, used
  // to collect final paths at the end
  Path<ThreadId> phase2_thread_path;

  // Only called by the Source thread to check if there is a path in the
  // collision graph from source to node
  void add_collision(ThreadId source, ThreadId target, Node node, Node parent);

  // Only called by the Source thread to check if there is a path in the
  // collision graph from source to node
  bool check_collision_path();

  // Called at each iteration of the search by all threads to check messages
  // from other threads
  FringeInterruptAction check_message_queue();

  // Initialize fringe search list, heuristic and source node cache entry.
  void initialize_fringe_search();

  // Called when a collision happens during the search
  void handle_collision(Node node, Node parent, ThreadId other);

  // Reset internal state for phase 2 iterations
  void reset_for_phase_2(Node source, Node target);

  // Reverse the path from 'from' to 'to using the node cache
  // and store it into final_path
  void finalize_path(Node from, Node to, bool include_to = true);

  // Entry point of all ripple threads
  void entry();

public:
  RippleThread(ThreadId id, Map &map, CollisionGraph &collision_graph,
               std::vector<RippleCacheNode> &cache,
               std::vector<concurrent_queue<Message>> &message_queues);

  bool start();
  bool join();

  // Setters for the source and goals variables
  void set_source(Node s);
  void set_single_goal(Node g);
  void set_goals(Node g1, Node g2);
  Path<ThreadId> &get_phase2_thread_path();
  Path<Node> &get_final_path();
};

// Utility class for initializing and invoking ripple search
class RippleSearch {
public:
  Map &map;
  std::vector<concurrent_queue<Message>> message_queues;
  std::vector<RippleCacheNode> cache;
  CollisionGraph collision_graph;
  Node source;
  Node goal;

  // Initialize search on a map
  RippleSearch(Map &map, Node source, Node goal);

  // Start the search from source to goal, they must be valid nodes
  std::optional<Path<Node>> search();
};
