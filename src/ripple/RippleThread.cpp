#include <cassert>
#include <memory>
#include <limits>

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
  Logf("Attempting to finalize path from %d to %d", from, to);
  Node current = from;

  while (current != to) {
    final_path.push_back(current);
    current = NODE_PARENT(cache[current].thread_parent.load(std::memory_order_seq_cst));
  }

  if (include_to) {
    final_path.push_back(to);
  }

  Log("Finished finalizing path");
}

// Only called by the Source thread to check if there is a path in the collision
// graph from source to node

// Called at each iteration of the search by all threads to check messages from
// other threads
FringeInterruptAction RippleThread::check_message_queue() {
  FringeInterruptAction response_action = NONE;
  Message message{};

  if (recv_message(message)) {
    switch (message.type) {
      // Only arrives to worker threads if they need to switch to phase 2
      // and to the goal thread to know that it is done
      // does not arrive to source thread
      case MESSAGE_PHASE_2: {
        Log("Message - Phase2");
        if (id == THREAD_SOURCE || id == THREAD_GOAL) {
          if(message.final_info.extra != -1) {
            final_path.push_back(message.final_info.extra);
          }

          if(!(id == THREAD_SOURCE && message.final_info.node == source)) {
            finalize_path(message.final_info.node, source, true);
          }
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

// Called when a collision happens during the search
void RippleThread::handle_collision(Node node, Node parent, ThreadId other) {
  assert(other != id);

  // check if we already collided with this thread, otherwise register the
  // collision
  if (collision_mask[other])
    return;

  // For worker threads update the heuristic
  if (goal_2 && collision_mask.none()) {
    float dist1 = map.distance(node, goal);
    float dist2 = map.distance(node, goal_2.value());

    if(dist1 < dist2) {
      //If we hit closer to goal
      heuristic = [this](Node n) {
        return map.distance(n, goal_2.value());
      };
    } else if(dist1 > dist2) {
      //If we hit closer to goal_2
      heuristic = [this](Node n) {
        return map.distance(n, goal);
      };
    }
  }

  collision_mask.set(other);

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
  if (phase == PHASE_1 && goal_2.has_value()){
    heuristic = [this] (Node n){
      return std::min(map.distance(n, goal), map.distance(n, goal_2.value()));
    };
  } else {
    // Relies on the fact that goal is closer than goal_2
    heuristic = [this](Node n) {
      return map.distance(n, goal);
    };
  }

  if (phase == PHASE_2 && source == goal) {
    return phase_2_conclusion(source);
  }

  std::vector<Node> now_list = { source };
  std::vector<Node> later_list;

  float flimit = heuristic(source);
  int current_list = 0; // 0 or 1

  cache[source].node.visited = true;
  cache[source].node.cost = 0;
  cache[source].node.list_index = current_list;
  cache[source].node.phase = phase;

  while (!now_list.empty()) {
    float fmin = std::numeric_limits<float>::max();

    while (!now_list.empty()) {
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

      Node node = now_list.back();
      now_list.pop_back();

      // Load info for the current node
      RippleNode &node_info = cache[node].node;

      if (node_info.list_index != current_list) {
        continue;
      }

      node_info.list_index = -1;

      // Compute fringe heuristic for current node
      float cost = node_info.cost;
      float f = cost + heuristic(node);

      // Skip node if fringe heuristic lower than the threshold
      if (f > flimit + 1) {
        fmin = std::min(fmin, f);
        later_list.push_back(node);
        node_info.list_index = 1 - current_list;
        continue;
      }

      Point point = map.node_to_point(node);

      // For each neighbour
      for (int i = 0; i < Map::NEIGHBOURS_COUNT; i++) {
        Point neighbor_point = point + Map::neighbour_offsets[i];

        if (!map.in_bounds(neighbor_point) || !map.get(neighbor_point)) {
          continue;
        }

        const Node neighbor = map.point_to_node(neighbor_point);

        // If we found our goal in phase 2 we are done and can exit;
        if (phase == PHASE_2 && neighbor == goal) {
          return phase_2_conclusion(node);
        }

        // Compute cost of path to s
        float cost_nb = i >= 4 ? cost + 1 : cost + sqrtf(2);

        // Check if already owned, otherwise try to acquire
        uint64_t thread_parent = cache[neighbor].thread_parent.load(std::memory_order_relaxed);
        ThreadId owner = NODE_OWNER(thread_parent);

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
                !cache[neighbor].thread_parent.compare_exchange_strong(thread_parent, MAKE_OWNER_PARENT(id, node)))) {
            // If we failed to acquire the node we need to handle the
            // collision
            handle_collision(neighbor, node, NODE_OWNER(thread_parent));
            continue;
          }
        }

        // Skip neighbor if already visited in this phase with a lower cost
        RippleNode& neighbor_cache = cache[neighbor].node;
        if (neighbor_cache.phase == phase && neighbor_cache.visited &&
            cost_nb >= neighbor_cache.cost) {
          continue;
        }

        // Update neighbour cache entry
        neighbor_cache.visited = true;
        neighbor_cache.cost = cost_nb;
        neighbor_cache.phase = phase;

        if (cache[neighbor].node.list_index != current_list) {
          now_list.push_back(neighbor);
          cache[neighbor].node.list_index = current_list;
        }

        // Update neighbour parent
        cache[neighbor].thread_parent = MAKE_OWNER_PARENT(id, node);
      }
    }

    std::swap(now_list, later_list);
    current_list = 1 - current_list;

    // Update fringe search threshold with the minimum value present in the new list
    flimit = fmin;
  }

  if (phase == PHASE_2) {
    Logf("Didn't find goal in phase 2! %d (%d) -> %d (%d)",
         source, NODE_OWNER(cache[source].thread_parent.load()),
         goal, NODE_OWNER(cache[goal].thread_parent.load()));
    assert(false);
  }

  return phase_1_conclusion();
}

void RippleThread::phase_1_conclusion() {
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

void RippleThread::phase_2_conclusion(Node parent_of_goal) {
  Log("Worker finish");
  final_path.push_back(goal);
  finalize_path(parent_of_goal, source, id == THREAD_GOAL);
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