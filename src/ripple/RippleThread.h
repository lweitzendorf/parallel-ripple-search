#pragma once

#include "Collision.h"
#include "Message.h"
#include "Thread.h"
#include "graph/map.h"
#include "reference/fringe.h"
#include "utility/Timer.h"

#include <atomic>
#include <vector>

#include <oneapi/tbb/concurrent_queue.h>

using oneapi::tbb::concurrent_queue;

#define LOG_ENABLED false

#if LOG_ENABLED
#define Log(str) printf("%d|" str "\n", id)
#define LogNOID(str) printf(str "\n")
#define Logf(fmt, ...) printf("%d|" fmt "\n", id, __VA_ARGS__)
#define LogfNOID(fmt, ...) printf(fmt "\n", __VA_ARGS__)
#define AssertUnreachable(...)                                                 \
  do {                                                                         \
    LogNOID(__VA_ARGS__);                                                      \
    assert(false);                                                             \
  } while (0)
#else
#define Log(...)
#define LogNOID(...)
#define Log(...)
#define LogfNOID(...)
#define AssertUnreachable(...)
#endif

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
  // Worker threads have 2 goals because they search in both directions
  // until they have a collision with one of the two.
  Node source;
  Node goal;
  std::optional<Node> goal_2;

  // Fringe search info
  FringeList fringe_list;
  std::function<int(RippleThread *, Node)> heuristic;
  int flimit;

  // Reference to vector of node info for fringe search, shared between all
  // threads
  std::vector<RippleCacheNode> &cache;

  // Mask of threads that we collided with, the i-th bit is 1 if we collided
  // with the i-th thread
  uint32_t collision_mask = 0;
  static_assert(sizeof(collision_mask) * 8 >= NUM_SEARCH_THREADS);

  // Phase 2 flag, set to true when we enter phase 2
  bool phase2 = false;

  // Segment of the final path owned by the thread
  Path<Node> final_path;

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

  void send_message(Message &msg);
  bool recv_message(Message &msg);

  // Entry point of all ripple threads
  void entry();

  void search();

  void phase_1_conclusion();
  void phase_2_conclusion();
  void exit();

public:
  RippleThread(
      ThreadId id, Map &map, std::vector<RippleCacheNode> &cache,
      std::vector<tbb::detail::d2::concurrent_queue<Message>> &message_queues);

  bool start();
  bool join();

  // Setters for the source and goals variables
  void set_source(Node s);
  void set_single_goal(Node g);
  void set_goals(Node g1, Node g2);
  Path<Node> &get_final_path();
};
