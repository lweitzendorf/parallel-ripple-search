#pragma once

#include "Collision.h"
#include "Message.h"
#include "Constants.h"
#include "graph/Map.h"
#include "reference/FringeSearch.h"
#include "utility/Timer.h"

#include <atomic>
#include <vector>

#include <oneapi/tbb/concurrent_queue.h>

using oneapi::tbb::concurrent_queue;

// Question: can we put some of these enums within the class? is this a better
// design?
enum FringeInterruptAction { NONE, RESET, EXIT };

struct RippleNode {
  // Fringe search data
  bool visited = false;
  float cost; // Cost from source to node
  int list_index = -1;
  Phase phase = PHASE_1;
};

static_assert(sizeof(ThreadId) <= 4 && sizeof(Node) <= 4);
#define NODE_PARENT(v) ((Node)(v & 0xFFFFFFFF))
#define NODE_OWNER(v) ((ThreadId)(v >> 32))
#define MAKE_OWNER_PARENT(owner, parent) ((uint64_t)owner << 32 | (uint64_t)parent)

struct RippleCacheNode {
  // Packed id and parent node for atomic update:
  // high 32 bits: id of the thread that first discovered the Node and thus owns it
  // low  32 bits: parent Node 
  std::atomic<uint64_t> thread_parent;
  RippleNode node;
};

// Class representing a thread of the ripple search algorithm
class RippleThread {
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

  // Current source ang goals for the thread.
  // Worker threads have 2 goals because they search in both directions
  // until they have a collision with one of the two.
  Node source;
  Node goal;
  std::optional<Node> goal_2;

  // Fringe search info
  std::function<float(Node)> heuristic;

  // Reference to vector of node info for fringe search, shared between all
  // threads
  std::vector<RippleCacheNode> &cache;

  // Mask of threads that we collided with, the i-th bit is 1 if we collided
  // with the i-th thread
  std::bitset<NUM_SEARCH_THREADS> collision_mask = 0;

  // Segment of the final path owned by the thread
  Path<Node> final_path;

  // Called at each iteration of the search by all threads to check messages
  // from other threads
  FringeInterruptAction check_message_queue();

  // Called when a collision happens during the search
  void handle_collision(Node node, Node parent, ThreadId other);

  // Reverse the path from 'from' to 'to using the node cache
  // and store it into final_path
  void finalize_path(Node to, Node from, bool include_to = true);

  void send_message(Message &msg);
  bool recv_message(Message &msg);

  // Entry point of all ripple threads
  void entry();

  void search(Phase phase);

  void phase_1_conclusion();
  void phase_2_conclusion(Node parent_of_goal);
  void exit();

public:
  RippleThread(
      ThreadId id, Map &map, std::vector<RippleCacheNode> &cache,
      std::vector<tbb::detail::d2::concurrent_queue<Message>> &message_queues);

  bool start();
  bool join();

  // Setters for the source and goals variables
  void set_src_and_goal(Node s, Node g);
  void set_src_and_goals(Node s, Node g1, Node g2);
  const Path<Node> &get_final_path() const;
};
