#include <cassert>
#include <iostream>
#include <memory>

#include "HighLevelGraph.h"
#include "RippleThread.h"

RippleThread::RippleThread(
    ThreadId id, Map &map, std::vector<RippleCacheNode> &cache,
    std::vector<tbb::detail::d2::concurrent_queue<Message>> &message_queues)
    : id(id), thread(nullptr), map(map), message_queues(message_queues),
      cache(cache) {}

void RippleThread::set_source(Node s) { source = s; }

void RippleThread::set_single_goal(Node g) {
  goal = g;
  goal_2 = {};
}

void RippleThread::set_goals(Node g1, Node g2) {
  goal = g1;
  goal_2 = g2;
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

// Called at each iteration of the search by all threads to check messages from
// other threads
FringeInterruptAction RippleThread::check_message_queue() {
  FringeInterruptAction response_action = NONE;
  Message message;

  while (recv_message(message)) {
    switch (message.type) {
    // Only arrives to slave threads if they need to switch to phase 2
    // and to the goal thread to know that he is done
    // does not arrive to source thread
    case MESSAGE_PHASE_2: {
      Log("Message - Phase2");

      if (id == THREAD_SOURCE) {
        finalize_path(message.final_node, source);
        response_action = EXIT;
      } else if (id == THREAD_GOAL) {
        finalize_path(message.final_node, source);
        response_action = EXIT;
      } else {
        reset_for_phase_2(message.phase_info.source, message.phase_info.target);
        response_action = RESET;
      }
    } break;

      // Only arrives to slave threads if they do not have to work in phase 2
    case MESSAGE_STOP:
      Log("Message - Stop");
      response_action = EXIT;
      break;

    default: {
      AssertUnreachable(
          "This kind of message should only be sent to the coordinator!");
    } break;
    }
  }
  return response_action;
}

void RippleThread::reset_for_phase_2(Node src, Node target) {
  set_source(src);
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

  // For worker threads update the heuristic and check if we are done
  if (id != THREAD_SOURCE && id != THREAD_GOAL) {
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

  // Message the coordinator thread about the collision
  Message message{
      .type = MESSAGE_COLLISION,
      .collision_info =
          {
              .collision_node = node,
              .collision_parent = parent,
              .collision_source = id,
              .collision_target = other,
          },
  };
  send_message(message);
}

void RippleThread::entry() { search(); }

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
            if (*node == goal ||
                (goal_2.has_value() && *node == goal_2.value())) {
              assert(false);
            }
          }

          // Compute cost of path to s
          int gs = g + 1;

          // Check if already owned, otherwise try to acquire
          ThreadId owner =
              cache[neighbour].thread.load(std::memory_order_relaxed);

          // In phase two we skip all nodes that are not owned by us
          if (phase2 && owner != id) {
            continue;
          }

          if (!phase2) {
            // We test the following things in order:
            // - if we don't already own the node
            // - if someone else owns the node or we failed to acquire the node
            // If any of these is true a collision has happened.
            // The order is important as we want to avoid the expensive compare
            // and swap if any of first two checks short circuits.
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

  // NOTE We can immediately jump here when a thread is supposed to stop what
  // it's
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
  Log("Worker finish");

  // Reconstruct the path
  if (cache[goal].node.visited)
    finalize_path(goal, source, false);

  Message msg{.type = MESSAGE_DONE};
  send_message(msg);
}

void RippleThread::exit() {
  Log("Thread Exiting");
  timer.stop();
  time_second = timer.get_microseconds() / 1000.0;

  Message msg{.type = MESSAGE_DONE};
  send_message(msg);
}

inline void RippleThread::send_message(Message &msg) {
  message_queues[THREAD_COORDINATOR].push(msg);
}

inline bool RippleThread::recv_message(Message &msg) {
  return message_queues[id].try_pop(msg);
}
