#include <memory>
#include <iostream>
#include <cassert>
#include "reference/Astar.h"
#include "HighLevelGraph.h"
#include "RippleThread.h"

RippleThread::RippleThread(
    ThreadId id, Map &map, CollisionGraph &collision_graph,
    std::vector<RippleCacheNode> &cache,
    std::vector<tbb::detail::d2::concurrent_queue<Message>> &message_queues)
    : id(id), thread(nullptr), map(map), message_queues(message_queues),
      cache(cache), collision_graph(collision_graph) {}

void RippleThread::set_source(Node s) { source = s; }

void RippleThread::set_single_goal(Node g) {
  goal = g;
  goal_2 = {};
}

void RippleThread::set_goals(Node g1, Node g2) {
  goal = g1;
  goal_2 = g2;
}

Path<ThreadId> &RippleThread::get_phase2_thread_path() {
  if (id != THREAD_SOURCE)
    AssertUnreachable(
        "attemp to get the collision path from a non-source thread");

  return phase2_thread_path;
}

Path<Node> &RippleThread::get_final_path() { return final_path; }

bool RippleThread::start() {
  if (thread == nullptr) {
    thread = std::make_unique<std::thread>(&RippleThread::entry, this);
    return true;
  }
  return false;
}

bool RippleThread::join() {
  if (thread != nullptr) {
    thread->join();
    thread = nullptr;
    return true;
  }
  return false;
}

void RippleThread::add_collision(ThreadId source, ThreadId target, Node node,
                                 Node parent) {
  if (id != THREAD_SOURCE)
    AssertUnreachable("add_collision called on non-source thread");

  collision_graph.add_collision(source, target, node, parent);
}

void RippleThread::finalize_path(Node from, Node to, bool include_to) {
  Log("Attempting to finalize path");

  Node current = from;
  while (current != to) {
    final_path.push_back(current);
    current = cache[current].node.parent;
  }

  if (include_to)
    final_path.push_back(to);

  Log("Finished finalizing path");
}

// Only called by the Source thread to check if there is a path in the collision
// graph from source to node
bool RippleThread::check_collision_path() {
  if (id != THREAD_SOURCE)
    AssertUnreachable("check_collision_graph called on non-source thread");

  auto maybe_path = a_star_search(collision_graph, THREAD_SOURCE, THREAD_GOAL);

  // If there is a path signal all threads to stop
  if (maybe_path) {

    timer.stop();
    time_first = timer.get_microseconds() / 1000.0;

    timer.start();

    phase2 = true;

    auto found_path = maybe_path.value();

    Log("Path found:");
#if LOG_ENABLED
    for (auto n : found_path) {
      printf("%d -> ", n);
    }
    printf("||\n");
#endif

    if (found_path.front() != THREAD_SOURCE)
      AssertUnreachable(
          "I expected the source thread to be the start of the path");
    if (found_path.back() != THREAD_GOAL)
      AssertUnreachable("I expected the goal thread to be the end of the path");

    // Send message with next source and target to all threads
    // that aren't the source / target thread.
    Message msg;

    bool works_in_phase2[NUM_THREADS] = {};

    // SLAVES WORKING
    // For each slave send target and source
    for (int i = 1; i < found_path.size() - 1; i++) {
      ThreadId thread = found_path[i];
      auto collision_with_prev =
          collision_graph.get_collision(found_path[i - 1], found_path[i]);
      // TODO we could make this /slightly/ faster by holding on to these values
      //      as the 'collision_with_prev' at a later point. (same for the
      //      collision_path values)
      auto collision_with_next =
          collision_graph.get_collision(found_path[i], found_path[i + 1]);

      msg.type = MESSAGE_PHASE_2;
      msg.source = collision_with_next.node;
      msg.target = collision_with_prev.node;

      // Set ownership of goal node to the current thread
      cache[msg.target].thread.store(thread, std::memory_order_seq_cst);

      message_queues[thread].push(msg);
      works_in_phase2[thread] = true;

      Logf("Phase2: SLAVE %d <- %d -> %d", found_path[i - 1], thread,
           found_path[i + 1]);
    }

    // SLAVES STOPPED
    // Stop all slaves that don't work in phase 2
    msg.type = MESSAGE_STOP;
    for (int i = 1; i < NUM_THREADS - 1; i++) {
      if (!works_in_phase2[i]) {
        message_queues[i].push(msg);
      }
    }

    // NOTE:
    // The order between source and goal here is important when the only
    // collision is the one between source and goal. In that case we
    // first store into current the node that source starts reversing from
    // and then update the node cache so that goal is able to reverse his.
    // This should probably be made more robust once we handle collisions
    // better.

    // SOURCE
    // Start reconstructing path from source to first
    Collision first_collision =
        collision_graph.get_collision(THREAD_SOURCE, found_path[1]);
    Node current = first_collision.parent;
    if (first_collision.target == THREAD_SOURCE) {
      current = cache[first_collision.node].node.parent;
    }

    // GOAL
    // Update the cache, so that the parent of the last collision is always a
    // node that was discovered by goal thread.
    ThreadId second_last = found_path[found_path.size() - 2];
    Logf("Phase2: GOAL -> %d", second_last);
    Collision last_collision =
        collision_graph.get_collision(second_last, THREAD_GOAL);

    if (last_collision.target != THREAD_GOAL) {
      cache[last_collision.node].thread.store(THREAD_GOAL,
                                              std::memory_order_seq_cst);
      cache[last_collision.node].node.parent = last_collision.parent;
    } else {
      assert(cache[last_collision.node].thread.load(
                 std::memory_order_seq_cst) == THREAD_GOAL);
    }

    // Signal the node to start reconstruction with to the goal thread
    msg.type = MESSAGE_PHASE_2;
    msg.final_node = last_collision.node;
    message_queues[THREAD_GOAL].push(msg);

    finalize_path(current, source);
    std::reverse(final_path.begin(), final_path.end());

    // Store the finalized path
    this->phase2_thread_path = found_path;

    // Wait for all slaves and the goal to end
    int slaves_working = found_path.size() - 1;
    Logf("Slaves working: %d", slaves_working);
    while (slaves_working) {
      // TODO: not busy wait
      // std::unique_lock<std::mutex> lk(wait_mutex);
      // wait_cv.wait(lk);

      while (message_queues[THREAD_SOURCE].try_pop(msg)) {
        if (msg.type == MESSAGE_DONE) {
          slaves_working--;
          Logf("Slaves working: %d", slaves_working);
        }
      }
    }

    timer.stop();
    time_second = timer.get_microseconds() / 1000.0;

    return true;
  } else {
    Log("Collision path not found");
  }

  return false;
}

// Called at each iteration of the search by all threads to check messages from
// other threads
FringeInterruptAction RippleThread::check_message_queue() {
  FringeInterruptAction response_action = NONE;
  bool check_collisions = false;
  Message message;
  while (message_queues[id].try_pop(message)) {
    switch (message.type) {

    // Only arrives to THREAD_SOURCE
    case MESSAGE_COLLISION: {
      Logf("Message - Collision: %d -> %d", message.collision_source,
           message.collision_target);
      add_collision(message.collision_source, message.collision_target,
                    message.collision_node, message.collision_parent);
      check_collisions = true;
    } break;
    // Only arrives to slave threads if they need to switch to phase 2
    // and to the goal thread to know that he is done
    // does not arrive to source thread
    case MESSAGE_PHASE_2: {
      Log("Message - Phase2");

      if (id == THREAD_GOAL) {
        timer.stop();
        time_first = timer.get_microseconds() / 1000.0;
        timer.start();

        finalize_path(message.final_node, source);
        Message msg;
        msg.type = MESSAGE_DONE;
        message_queues[THREAD_SOURCE].push(msg);
        response_action = EXIT;
      } else if (id == THREAD_SOURCE) {
        assert(false);
      } else {
        reset_for_phase_2(message.source, message.target);
        response_action = RESET;
      }
    } break;

      // Only arrives to slave threads if they do not have to work in phase 2
    case MESSAGE_STOP:
      Log("Message - Stop");

      response_action = EXIT;
      break;
    }
  }

  // The Source thread might need to check collisions
  if (check_collisions) {
    if (check_collision_path()) {
      response_action = EXIT;
    }
  }
  return response_action;
}

void RippleThread::reset_for_phase_2(Node source, Node target) {
  set_source(source);
  set_single_goal(target);
  phase2 = true;
}

// Initialize fringe search list, heuristic and source node cache entry.
void RippleThread::initialize_fringe_search() {
  // Initialize fring list with source node only
  fringe_list.clear();
  fringe_list.push_front(source);

  // Set source node cache entry
  cache[source].node.in_list = true;
  cache[source].node.g = 0;
  cache[source].node.visited = true;
  cache[source].node.list_entry = fringe_list.begin();
  cache[source].node.phase2 = phase2; // set to true if we are in phase 2

  // Set the heuristic depending on the numbe of goals
  if (!goal_2) {
    heuristic = [](RippleThread *self, Node n) {
      return self->map.distance(n, self->goal);
    };
  } else {
    heuristic = [](RippleThread *self, Node n) {
      return std::min(self->map.distance(n, self->goal),
                      self->map.distance(n, self->goal_2.value()));
    };
  }

  // Set fringe threshold
  flimit = heuristic(this, source);
}

// Called when a collision happens during the search
void RippleThread::handle_collision(Node node, Node parent, ThreadId other) {
  assert(other != id);

  // check if we already collided with this thread, otherwise register the
  // collision
  if (collision_mask & (1 << other)) {
    return;
  } else {
    collision_mask |= (1 << other);
  }

  // For slave threads update the heuristic and check if we are done
  if (id > THREAD_SOURCE && id < THREAD_GOAL) {
    // If we collided closer to the source we now only care about goal_2
    if (other < id) {
      heuristic = [](RippleThread *self, Node n) {
        return self->map.distance(n, self->goal_2.value());
      };
    } else {
      // Otherwise we now only care about goal
      heuristic = [](RippleThread *self, Node n) {
        return self->map.distance(n, self->goal);
      };
    }
  }

  // Message the source thread about the collision, unless we are the source
  // thread
  if (id != THREAD_SOURCE) {
    Message message;
    message.type = MESSAGE_COLLISION;
    message.collision_source = id;
    message.collision_target = other;
    message.collision_node = node;
    message.collision_parent = parent;
    message_queues[THREAD_SOURCE].push(message);
  } else {
    add_collision(THREAD_SOURCE, other, node, parent);
    check_collision_path();
  }
}


/** NOTE A few notes for refactoring:
 *
 * we need to get rid of the GOTO
 *  the best way to do this is with a continuation (to my knowledge)
 *  here's a [decent] article comparing this with Haskell:
 *  https://www.fpcomplete.com/blog/2012/06/asynchronous-api-in-c-and-the-continuation-monad/
 *
 *  obviously we do not need something this heavy, but I could imagine passing
 *  a few functions to the search function that will make this easier.
 *
 *  Oh, and pull everything out of the 'entry function. This should only be
 *  for setting up some thread specific things and /that's it/.
 */

void RippleThread::entry() {
  search();
}

void RippleThread::search() {
  timer.start();
  initialize_fringe_search();

  // Fringe search step towards G
  auto node = fringe_list.end();

  while (!fringe_list.empty()) {
    int fmin = INT_MAX;
    node = fringe_list.begin();

    do {
      switch (check_message_queue()) {
        case RESET:
          return search();
        case EXIT:
          return exit();
        case NONE:
          break;
    }

      // Load info for the current node
      FringeNode &node_info = cache[*node].node;

      // Compute fringe heuristic for current node
      int g = node_info.g;
      int f = g + heuristic(this, *node);

      // Skip node if fringe heuristic lower than the threshold
      if (f > flimit) {
        fmin = std::min(fmin, f);
        node++;
        continue;
      }

      Point np = map.node_to_point(*node);

      // For each neighbour
      for (auto &neighbour_offset : Map::neighbour_offsets) {
        Point neigh = np + neighbour_offset;

        // If the neighbour coordinates are inside the map
        if (map.in_bounds(neigh)) {
          Node neighbour = map.point_to_node(neigh);

          // If it's a wall skip
          if (!map.get(neigh)) {
            continue;
          }

          // If we found our goal in phase 2 we are done and can exit;
          if (phase2) {
            if (*node == goal) {
              return phase_2_conclusion();
            }
          } else {
            // This should never happen in phase 1 as the goal threads
            // are already acquired
            if (*node == goal || (goal_2.has_value() && *node == goal_2.value())) {
              assert(false);
            }
          }

          // Compute cost of path to s
          int gs = g + 1;

          // Check if already owned, otherwise try to acquire
          ThreadId owner = cache[neighbour].thread.load(std::memory_order_relaxed);

          // In phase two we skip all nodes that are not owned by us
          if (phase2 && owner != id) {
            continue;
          }

          if (!phase2) {
            // We test the following things in order:
            // - if we don't already own the node
            // - if someone else owns the node or we failed to acquire the node
            // If any of these is true a collision has happened.
            // The order is important as we want to avoid the expensive compare and swap
            // if any of first two checks short circuits.
            if (owner != id &&
                (owner != THREAD_NONE ||
                 !cache[neighbour].thread.compare_exchange_strong(
                     owner, id, std::memory_order_seq_cst,
                     std::memory_order_seq_cst))) {
              // If we failed to acquire the node we need to handle the
              // collision
              handle_collision(neighbour, *node, owner);

              // Skip the node
              continue;
            }
          }

          // Skip neighbour if already visited in this phase with a lower cost
          FringeNode &neighbour_cache = cache[neighbour].node;
          if (neighbour_cache.phase2 == phase2 && neighbour_cache.visited &&
              gs >= neighbour_cache.g) {
            continue;
          }

          // If already in list in this phase, remove it
          if (neighbour_cache.phase2 == phase2 && neighbour_cache.in_list) {
            fringe_list.erase(neighbour_cache.list_entry);
            neighbour_cache.in_list = false;
          }

          // Insert neighbour in the list right after the current node
          fringe_list.insert(std::next(node), neighbour);

          // Update neighbour cache entry
          neighbour_cache.visited = true;
          neighbour_cache.g = gs;
          neighbour_cache.parent = *node;
          neighbour_cache.list_entry = std::next(node);
          neighbour_cache.in_list = true;
          neighbour_cache.phase2 = phase2;
        }
      }

      // Update current node cache entry
      node_info.in_list = false;

      // Remove current node from the list
      auto tmp = std::next(node);
      fringe_list.erase(node);
      node = tmp;
    } while (node != fringe_list.end());

    // Update fring search threshold with the minimum value present in the new
    // list
    flimit = fmin;
  }

  // TODO this should not happen in phase 2
  return phase2 ? phase_2_conclusion() : phase_1_conclusion();

// NOTE We can immediately jump here when a thread is supposed to stop what it's
//      doing. This is considered an interrupt action.
}

void RippleThread::phase_1_conclusion() {
  timer.stop();
  time_first = timer.get_microseconds() / 1000.0;

  // If we had no collision (TODO: we could also check if we are a slave with
  // only 1 collision)

  if (collision_mask == 0) {
    // If we are the source or goal thread abort the search
    if (id == THREAD_SOURCE || id == THREAD_GOAL) {
      AssertUnreachable("Path does not exist, currently not handled");
    } else {
      return exit();
    }
  } else {
    // TODO Wait for messages and then go back to reset
    Log("Waiting");
    while (true) {
      switch (check_message_queue()) {
        case RESET:
          return search();
        case EXIT:
          return exit();
        case NONE:
          break;
      }
    }
  }
}

void RippleThread::phase_2_conclusion() {
  if (id == THREAD_SOURCE || id == THREAD_GOAL) {
    ; // FIXME what goes here?
  } else {
    Logf("Slave finish - found: %d", found);

    // Reconstruct the path
    if (cache[goal].node.visited)
      finalize_path(goal, source, false);

    Message msg;
    msg.type = MESSAGE_DONE;
    message_queues[THREAD_SOURCE].push(msg);
  }
}

void RippleThread::exit() {
  Log("Thread Exiting");
  timer.stop();
  time_second = timer.get_microseconds() / 1000.0;
}