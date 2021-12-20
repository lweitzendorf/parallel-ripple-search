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
// TODO: test aligning this struct to cache line size to avoid
// false sharing between threads that are exploring neighbouring nodes

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
  std::function<int(Node)> heuristic;

  std::vector<FringeNode> cache;
  std::vector<std::atomic<ThreadId>> &node_owners;

  // Mask of threads that we collided with, the i-th bit is 1 if we collided
  // with the i-th thread
  uint32_t collision_mask = 0;
  static_assert(sizeof(collision_mask) * 8 >= NUM_SEARCH_THREADS);

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
  void phase_2_conclusion();
  void exit();

public:
  RippleThread(
      ThreadId id, Map &map, std::vector<std::atomic<ThreadId>> &node_owners,
      std::vector<tbb::detail::d2::concurrent_queue<Message>> &message_queues);

  bool start();
  bool join();

  // Setters for the source and goals variables
  void set_src_and_goal(Node s, Node g);
  void set_src_and_goals(Node s, Node g1, Node g2);
  const Path<Node> &get_final_path() const;
};
