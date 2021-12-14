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

void RippleThread::set_src_and_goal(Node s, Node g) {
  source = s;
  goal = g;
  goal_2 = {};
}

void RippleThread::set_src_and_goals(Node s, Node g1, Node g2) {
  source = s;
  float d1 = map.distance(source, g1);
  float d2 = map.distance(source, g2);
  goal = (d1 < d2 ? g1 : g2);
  goal_2 = (d1 < d2 ? g2 : g1);
}

const Path<Node> &RippleThread::get_final_path() const { return final_path; }

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
  do  {
    final_path.push_back(current);
    current = cache[current].node.parent;
  } while (current != to && current != from);

  if (current == from) {
    std::cout << "Thread " << id << ": detected cycle of size " << final_path.size() << "!" << std::endl;
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
      // Only arrives to worker threads if they need to switch to phase 2
      // and to the goal thread to know that it is done
      // does not arrive to source thread
      case MESSAGE_PHASE_2: {
        Log("Message - Phase2");
        if (id == THREAD_SOURCE || id == THREAD_GOAL) {
          finalize_path(message.final_node, source);
          response_action = EXIT;
        } else {
          set_src_and_goal(message.phase_info.source, message.phase_info.target);
          response_action = RESET;
        }
      } break;
      // Only arrives to worker threads if they do not have to work in phase 2
      case MESSAGE_STOP:
        Log("Message - Stop");
        response_action = EXIT;
        break;

      default: {
        AssertUnreachable("This kind of message should only be sent to the coordinator!");
      }
    }
  }
  return response_action;
}

// Initialize fringe search list, heuristic and source node cache entry.
void RippleThread::initialize_fringe_search(Phase phase) {
  // Initialize fring list with source node only
  fringe_list.clear();
  fringe_list.push_front(source);

  // Set source node cache entry
  cache[source].node.in_list = true;
  cache[source].node.g = 0;
  cache[source].node.visited = true;
  cache[source].node.list_entry = fringe_list.begin();
  cache[source].node.phase2 = phase; // set to true if we are in phase 2

  // Relies on the fact that goal is closer than goal_2
  heuristic = [this](Node n) {
    return map.distance(n, goal);
  };

  // Set fringe threshold
  flimit = heuristic(source);
}

// Called when a collision happens during the search
void RippleThread::handle_collision(Node node, Node parent, ThreadId other) {
  assert(other != id);

  // check if we already collided with this thread, otherwise register the
  // collision
  if (collision_mask & (1 << other))
    return;
  else
    collision_mask |= (1 << other);

  // For worker threads update the heuristic
  if (goal_2 && collision_mask == (1 << other)) {
    heuristic = [this](Node n) {
        return map.distance(n, goal_2.value());
    };
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

void RippleThread::entry() { search(PHASE_1); }

void RippleThread::search(Phase phase) {
  timer.start();
  initialize_fringe_search(phase);

  // Fringe search step towards G
  auto node = fringe_list.end();

  while (!fringe_list.empty()) {
    int fmin = INT_MAX;
    node = fringe_list.begin();

    do {
      switch (check_message_queue()) {
        case RESET:
          return search(PHASE_2);
        case EXIT:
          return exit();
        case NONE:
          break;
      }

      // If we found our goal in phase 2 we are done and can exit;
      if (phase == PHASE_2) {
        if (*node == goal)
          return phase_2_conclusion();
      } else if (*node == goal || (goal_2.has_value() && *node == goal_2.value())) {
        AssertUnreachable("Goal nodes should already be acquired in phase 1!");
      }

      // Load info for the current node
      FringeNode &node_info = cache[*node].node;

      // Compute fringe heuristic for current node
      int g = node_info.g;
      int f = g + heuristic(*node);

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

        if (!map.in_bounds(neigh) || !map.get(neigh)) {
          continue;
        }

        Node neighbour = map.point_to_node(neigh);

        // Compute cost of path to s
        int gs = g + 1;

        // Check if already owned, otherwise try to acquire
        ThreadId owner =
            cache[neighbour].thread.load(std::memory_order_relaxed);

        // In phase two we skip all nodes that are not owned by us
        if (phase == PHASE_2 && owner != id) {
          continue;
        }

        if (phase == PHASE_1) {
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
        if (neighbour_cache.phase2 == phase && neighbour_cache.visited &&
            gs >= neighbour_cache.g) {
          continue;
        }

        // If already in list in this phase, remove it
        if (neighbour_cache.phase2 == phase && neighbour_cache.in_list) {
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
        neighbour_cache.phase2 = phase;
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
  return phase == PHASE_1 ? phase_1_conclusion() : phase_2_conclusion();

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
          return search(PHASE_2);
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
