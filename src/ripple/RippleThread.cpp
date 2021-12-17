#include <cassert>
#include <iostream>
#include <memory>
#include <limits>

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

    //if (current != to)
    //  assert(cache[current].thread.load() == id);
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

  if (recv_message(message)) {
    switch (message.type) {
      // Only arrives to worker threads if they need to switch to phase 2
      // and to the goal thread to know that it is done
      // does not arrive to source thread
      case MESSAGE_PHASE_2: {
        Log("Message - Phase2");
        if (/* id == THREAD_GOAL */ false) {
          finalize_path(message.final_node, source);
          response_action = EXIT;
        } else {
          set_src_and_goal(message.path_info.source, message.path_info.target);
          response_action = RESET;
        }
      } break;
      // Only arrives to worker threads if they do not have to work in phase 2
      case MESSAGE_STOP: {
        Log("Message - Stop");
        response_action = EXIT;
      } break;

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

  if (phase == PHASE_2) {
    assert(cache[goal].thread.load() == id);
  }

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

  // For worker threads update the heuristic
  if (goal_2 && !collision_mask) {
    heuristic = [this](Node n) {
        return map.distance(n, goal_2.value());
    };
  }

  collision_mask |= (1 << other);

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

  while (!fringe_list.empty()) {
    int fmin = std::numeric_limits<int>::max();

    for (auto node = fringe_list.begin(); node != fringe_list.end(); node++) {
      switch (check_message_queue()) {
        case RESET:
          assert(phase == PHASE_1);
          return search(PHASE_2);
        case EXIT:
          assert(phase == PHASE_1);
          return exit();
        case NONE:
          break;
      }

      // If we found our goal in phase 2 we are done and can exit;
      if (phase == PHASE_2 && *node == goal) {
        return phase_2_conclusion();
      }

      // Load info for the current node
      FringeNode &node_info = cache[*node].node;

      // Compute fringe heuristic for current node
      int cost = (*node == source ? 0 : node_info.cost);
      int f = cost + heuristic(*node);

      // Skip node if fringe heuristic lower than the threshold
      if (f > flimit) {
        fmin = std::min(fmin, f);
        continue;
      }

      Point point = map.node_to_point(*node);

      // For each neighbour
      for (auto &neighbour_offset : Map::neighbour_offsets) {
        Point neighbor_point = point + neighbour_offset;

        if (!map.in_bounds(neighbor_point) || !map.get(neighbor_point)) {
          continue;
        }

        const Node neighbor = map.point_to_node(neighbor_point);

        if (neighbor == source) {
          continue;
        }

        // Compute cost of path to s
        int cost_nb = cost + 1;

        // Check if already owned, otherwise try to acquire
        ThreadId owner =
            cache[neighbor].thread.load(std::memory_order_relaxed);

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
               !cache[neighbor].thread.compare_exchange_strong(owner, id))) {
            // If we failed to acquire the node we need to handle the
            // collision
            handle_collision(neighbor, *node, owner);

            // Skip the node
            continue;
          }
        }

        // Skip neighbor if already visited in this phase with a lower cost
        FringeNode &neighbor_cache = cache[neighbor].node;
        if (neighbor_cache.phase == phase && neighbor_cache.visited &&
            cost_nb >= neighbor_cache.cost) {
          continue;
        }

        // If already in list in this phase, remove it
        if (neighbor_cache.phase == phase && neighbor_cache.in_list) {
          assert(int(std::distance(node, neighbor_cache.list_entry)) > 0);
          fringe_list.erase(neighbor_cache.list_entry);
          neighbor_cache.in_list = false;
        }

        // Insert neighbour in the list right after the current node
        fringe_list.insert(std::next(node), neighbor);

        // Update neighbour cache entry
        neighbor_cache.visited = true;
        neighbor_cache.cost = cost_nb;
        neighbor_cache.parent = *node;
        neighbor_cache.list_entry = std::next(node);
        neighbor_cache.in_list = true;
        neighbor_cache.phase = phase;
      }
      // Update current node cache entry
      node_info.in_list = false;

      // Remove current node from the list
      node = fringe_list.erase(node);
      node--;
    }

    // Update fringe search threshold with the minimum value present in the new
    // list
    flimit = fmin;
  }

  if (phase == PHASE_2) {
    std::cout << "Thread " << id << " didn't find goal in phase 2!" << std::endl;
    assert(false);
  }

  return phase_1_conclusion();
}

void RippleThread::phase_1_conclusion() {
  timer.stop();
  time_first = timer.get_microseconds() / 1000.0;

  Log("Waiting");
  Message msg {
    .type = MESSAGE_WAITING,
    .id = id,
  };
  send_message(msg);

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

void RippleThread::phase_2_conclusion() {
  Log("Worker finish");
  finalize_path(goal, source, id == THREAD_GOAL);
  return exit();
}

void RippleThread::exit() {
  Log("Thread Exiting");
  Message msg { .type = MESSAGE_DONE };
  send_message(msg);
}

inline void RippleThread::send_message(Message &msg) {
  message_queues[THREAD_COORDINATOR].push(msg);
}

inline bool RippleThread::recv_message(Message &msg) {
  return message_queues[id].try_pop(msg);
}
